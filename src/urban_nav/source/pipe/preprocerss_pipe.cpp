/**
 * @file    preprocerss_pipe.cpp
 * @brief   preprocerss pipeline
 * @author  niu_wengang@163.com
 * @date    2024-04-09 update
 * @version 0.1.1
 */

#include "preprocerss_pipe.hpp"

/**
 * @brief pipe init
 * @param[in] nh handle
 * @return
 */
PreProcessPipe::PreProcessPipe(ros::NodeHandle &nh)
{
    /*[1]--load params*/
    /*a--acquire yaml node*/
    paramlist_.package_folder_path = ros::package::getPath("urban_nav");
    YAML::Node config_node = YAML::LoadFile(paramlist_.package_folder_path + "/config/preprocerss.yaml");
    paramlist_.model_file_path = paramlist_.package_folder_path + "/model//pointpillar.onnx";
    /*b--get subscriber topic name*/
    paramlist_.imu_sub_topic = config_node["topic_sub"]["imu_sub_topic"].as<std::string>();
    paramlist_.gnss_sub_topic = config_node["topic_sub"]["gnss_sub_topic"].as<std::string>();
    paramlist_.cloud_sub_topic = config_node["topic_sub"]["cloud_sub_topic"].as<std::string>();
    /*C--get transformational matrix among sensors*/
    paramlist_.lidar_to_body = Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(
        config_node["extrinsic"]["lidar_to_body"].as<std::vector<float>>().data());
    paramlist_.imu_to_body = Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(
        config_node["extrinsic"]["imu_to_body"].as<std::vector<float>>().data());

    /*[2]--subscriber and publisher init*/
    imu_sub_ptr_ = std::make_shared<Tools::ImuSub>(nh, paramlist_.imu_sub_topic);
    cloud_sub_ptr_ = std::make_shared<Tools::CloudSub>(nh, paramlist_.cloud_sub_topic);
    gnss_sub_ptr_ = std::make_shared<Tools::GnssSub>(nh, paramlist_.gnss_sub_topic);

    imu_pub_ptr_ = std::make_shared<Tools::ImuPub>(nh, "synced_imu", "map");
    ground_cloud_pub_ptr_ = std::make_shared<Tools::CloudPub>(nh, "synced_ground_cloud", "map", 1); // ground cloud
    no_ground_cloud_pub_ptr_ =
        std::make_shared<Tools::CloudPub>(nh, "synced_no_ground_cloud", "map", 1);                    // no ground cloud
    dynamic_cloud_pub_ptr_ = std::make_shared<Tools::CloudPub>(nh, "synced_dynamic_cloud", "map", 1); // dynamic cloud

    gnss_pub_ptr_ = std::make_shared<Tools::OdomPub>(nh, "synced_gnss", "map", "gnss");
    bbx_pub_ptr_ = std::make_shared<Tools::BbxPub>(nh, "ods", "map");

    /*[3]--system debuger init*/
    log_ptr_ = std::make_shared<Tools::LogRecord>(paramlist_.package_folder_path + "/log", "preprocess");
    time_ptr_ = std::make_shared<Tools::TimeRecord>();

    /*[4]--module init*/
    gnss_odom_ptr_ = std::make_shared<Module::GnssOdom>();
    object_detection_ptr_ = std::make_shared<Module::ObjectDetection>(paramlist_.model_file_path);
    ground_seg_ptr_ = std::make_shared<Module::DipgGroundSegment>();
    spdlog::info("preprocerss_pipe$ inited");

    /*[5]--reset*/
    ground_cloud_ptr_.reset(new CloudMsg::CLOUD());
    no_ground_cloud_ptr_.reset(new CloudMsg::CLOUD());
    dynamic_cloud_ptr_.reset(new CloudMsg::CLOUD());
    static_cloud_ptr_.reset(new CloudMsg::CLOUD());
}

/**
 * @brief run
 * @param[in] none
 * @return
 */
bool PreProcessPipe::Run()
{
    /*[1]--read buffer*/
    if (!ReadMsgBuffer())
    {
        return false;
    }
    /*[2]--gnss init*/
    if (!gnss_msg_queue_.empty())
    {
        if (!gnss_odom_ptr_->InitPose(gnss_msg_queue_.front()))
        {
            return false;
        }
    }

    while (CheckMsgQueue())
    {
        if (!ReadMsg())
        {
            continue;
        }
        gnss_odom_ptr_->UpdateOdom(gnss_odom_, cur_gnss_msg_, cur_imu_msg_);
        gnss_odom_ =
            paramlist_.lidar_to_body.inverse() * paramlist_.imu_to_body * gnss_odom_; // transform to lidar frame

        time_ptr_->Start();

        object_detection_ptr_->Detect(cur_cloud_msg_, ods_msg_);
        ground_seg_ptr_->Segement(cur_cloud_msg_.cloud_ptr, ground_cloud_ptr_, no_ground_cloud_ptr_);

        DorPost(ods_msg_, no_ground_cloud_ptr_, static_cloud_ptr_, dynamic_cloud_ptr_);

        log_ptr_->terminal_->info("exec hz is:{}", time_ptr_->End(10e3));

        PublishMsg();
    }
    return true;
}

/**
 * @brief read buffer
 * @param[in]
 * @return if sync success
 * @todo if gnss lose,how to handle
 */
bool PreProcessPipe::ReadMsgBuffer()
{
    static std::deque<ImuMsg> unsynced_imu_msg_queue;
    static std::deque<GnssMsg> unsynced_gnss_msg_queue;

    /*[1]--read buffer*/
    cloud_sub_ptr_->ParseData(cloud_msg_queue_);
    imu_sub_ptr_->ParseData(unsynced_imu_msg_queue);
    gnss_sub_ptr_->ParseData(unsynced_gnss_msg_queue);

    if (cloud_msg_queue_.size() == 0)
    {
        return false;
    }
    /*[2]--sync queue*/
    const double refer_time = cloud_msg_queue_.front().time_stamp; // use cloud time as refer time

    bool valid_imu_flag = ImuMsg::TimeSync(unsynced_imu_msg_queue, imu_msg_queue_, refer_time);
    bool valid_gnss_flag = GnssMsg::TimeSync(unsynced_gnss_msg_queue, gnss_msg_queue_, refer_time);

    /*[3]--sync init*/
    static bool sensor_inited = false;
    if (!sensor_inited)
    {
        if (!valid_imu_flag or !valid_gnss_flag)
        {
            cloud_msg_queue_.pop_front();
            return false;
        }
        sensor_inited = true;
    }

    return true;
}

/**
 * @brief check msg queue
 * @param[in]
 * @return
 */
bool PreProcessPipe::CheckMsgQueue()
{
    if (gnss_msg_queue_.size() == 0)
    {
        return false;
    }

    if (cloud_msg_queue_.size() == 0)
    {
        return false;
    }

    if (imu_msg_queue_.size() == 0)
    {
        return false;
    }

    return true;
}

/**
 * @brief  read current message
 * @param[in]
 * @return
 */
bool PreProcessPipe::ReadMsg()
{

    cur_cloud_msg_ = cloud_msg_queue_.front();
    cur_imu_msg_ = imu_msg_queue_.front();
    cur_gnss_msg_ = gnss_msg_queue_.front();

    const double refer_time =
        std::max(std::max(cur_cloud_msg_.time_stamp, cur_gnss_msg_.time_stamp), cur_imu_msg_.time_stamp);

    const double cloud_refer_timediff = fabs(refer_time - cur_cloud_msg_.time_stamp);
    const double imu_refer_timediff = fabs(refer_time - cur_imu_msg_.time_stamp);
    const double gnss_refer_timediff = fabs(refer_time - cur_gnss_msg_.time_stamp);

    if (cloud_refer_timediff < 0.05 and imu_refer_timediff < 0.05 and gnss_refer_timediff < 0.05)
    {
        cloud_msg_queue_.pop_front();
        imu_msg_queue_.pop_front();
        gnss_msg_queue_.pop_front();
        return true;
    }
    else
    {
        if (cloud_refer_timediff > 0.05) // cloud delay
        {
            cloud_msg_queue_.pop_front();
        }
        if (imu_refer_timediff > 0.05) // imu delay
        {
            imu_msg_queue_.pop_front();
        }
        if (gnss_refer_timediff > 0.05) // gnss delay
        {
            gnss_msg_queue_.pop_front();
        }
        return false;
    }
}

/**
 * @brief publish message
 * @param[in]
 * @return
 */
void PreProcessPipe::PublishMsg()
{

    double refer_time = cur_cloud_msg_.time_stamp;

    no_ground_cloud_pub_ptr_->Publish(static_cloud_ptr_, refer_time);
    ground_cloud_pub_ptr_->Publish(ground_cloud_ptr_, refer_time);
    dynamic_cloud_pub_ptr_->Publish(dynamic_cloud_ptr_, refer_time);

    gnss_pub_ptr_->Publish(gnss_odom_, refer_time);
    bbx_pub_ptr_->Publish(ods_msg_, refer_time);

    // imu_pub_ptr_->Publish(cur_imu_msg_); // reserve
    // spdlog::info("preprocerss_pipe$ timestamp:{}", cur_cloud_msg_.time_stamp);
}

void PreProcessPipe::DorPost(const OdsMsg &ods_msg, const CloudMsg::CLOUD_PTR &cloud_ptr,
                             CloudMsg::CLOUD_PTR &static_cloud_ptr,
                             CloudMsg::CLOUD_PTR &dynamic_cloud_ptr) // dynamic object removal
{
    static_cloud_ptr.reset(new CloudMsg::CLOUD());
    dynamic_cloud_ptr.reset(new CloudMsg::CLOUD());

    for (const auto od : ods_msg.ods_queue)
    {
        pcl::CropBox<CloudMsg::POINT> clipper;

        Eigen::Vector4f minPoint(-od.w * 0.5, -od.l * 0.5, -od.h * 0.5, 1.0);
        Eigen::Vector4f maxPoint(od.w * 0.5, od.l * 0.5, od.h * 0.5, 1.0);

        clipper.setMin(minPoint);
        clipper.setMax(maxPoint);

        clipper.setTranslation(Eigen::Vector3f(od.x, od.y, od.z));
        clipper.setRotation(Eigen::Vector3f(0, 0, od.heading));

        CloudMsg::CLOUD_PTR outliers_cloud_ptr(new CloudMsg::CLOUD());
        CloudMsg::CLOUD_PTR inliers_cloud_ptr(new CloudMsg::CLOUD());

        clipper.setInputCloud(cloud_ptr);

        clipper.setNegative(false);
        clipper.filter(*inliers_cloud_ptr);
        *dynamic_cloud_ptr += *inliers_cloud_ptr;

        clipper.setNegative(true);
        clipper.filter(*outliers_cloud_ptr);
        *static_cloud_ptr += *outliers_cloud_ptr;
    }
}

// pcl::ExtractIndices<pcl::PointXYZ> extract;
// extract.setInputCloud(cloud_ptr);
// extract.setIndices(boost::make_shared<pcl::Indices>(indices));
// extract.setNegative(false); // inline

// extract.filter(*inliers_cloud_ptr);

// extract.setNegative(true);

// extract.filter(*outliers_cloud_ptr);
