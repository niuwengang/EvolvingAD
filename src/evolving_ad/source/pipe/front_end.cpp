/**
 * @file    front_end_pipe.cpp
 * @brief   loc front end pipeline
 * @author  niu_wengang@163.com
 * @date    2024-04-28
 * @version 0.1.4
 */

#include "front_end.hpp"

namespace evolving_ad_ns
{

/**
 * @brief FrontEndPipe init config
 * @param[in] nh
 * @param[in] package_folder_path
 * @return
 */
FrontEndPipe::FrontEndPipe(ros::NodeHandle &nh, const std::string package_folder_path)
{
    /*1--load params*/
    paramlist_.package_folder_path = package_folder_path;
    YAML::Node config_node = YAML::LoadFile(paramlist_.package_folder_path + "/config/ad.yaml");
    paramlist_.cloud_sub_topic = config_node["topic_sub"]["cloud_sub_topic"].as<std::string>();
    paramlist_.imu_sub_topic = config_node["topic_sub"]["imu_sub_topic"].as<std::string>();
    paramlist_.T_lidar2imu_ = Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(
        config_node["Extrinsic"]["T_lidar2imu"].as<std::vector<float>>().data());
    paramlist_.model_file_path = paramlist_.package_folder_path + "/model//pointpillar.onnx";

    /*2--topic sub */
    cloud_sub_ptr_ = std::make_shared<CloudSub>(nh, paramlist_.cloud_sub_topic);
    imu_sub_ptr_ = std::make_shared<ImuSub>(nh, paramlist_.imu_sub_topic);

    /*3--topic pub */
    static_cloud_pub_ptr_ = std::make_shared<CloudPub>(nh, "static_cloud", "map");
    ground_cloud_pub_ptr_ = std::make_shared<CloudPub>(nh, "ground_cloud", "map");
    dynamic_cloud_pub_ptr_ = std::make_shared<CloudPub>(nh, "dynamic_cloud", "map");
    bbx_pub_ptr_ = std::make_shared<BbxPub>(nh, "bbx", "map");
    veh_tf_pub_ptr_ = std::make_shared<TfPub>("map", "ground_link");
    lidar_odom_pub_ptr_ = std::make_shared<OdomPub>(nh, "lidar_odom", "map", "lidar");

    /*4--algorithm module*/
    object_detect_ptr_ = std::make_shared<ObjectDetect>(paramlist_.model_file_path);
    object_track_ptr_ = std::make_shared<ObjectTrack>();
    lidar_odom_ptr_ = std::make_shared<LidarOdom>(config_node["lidar_odom"]);
    imu_odom_ptr_ = std::make_shared<ImuOdom>(paramlist_.T_lidar2imu_);
    ground_seg_ptr_ = std::make_shared<DipgGroundSegment>();

    /*5--tools*/
    time_record_ptr_ = std::make_shared<TimeRecord>();

    /*6--log*/
    std::thread::id threadId = std::this_thread::get_id();
    spdlog::info("FrontEnd$ thread id:{}", *(std::thread::native_handle_type *)(&threadId));
    spdlog::info("FrontEnd$ init success");
}

/**
 * @brief FrontEndPipe run
 * @param[in]
 * @return
 */
bool FrontEndPipe::Run()
{
    /*1--read data*/
    cloud_sub_ptr_->ParseMsg(cloud_msg_queue_);
    imu_sub_ptr_->ParseData(imu_msg_queue_);

    if (cloud_msg_queue_.empty() == true)
    {
        return false;
    }

    /*2--load infomation into frame*/
    time_record_ptr_->Start();
    /*2.a--time stamp record*/
    current_frame_.time_stamp = cloud_msg_queue_.front().time_stamp;
    /*2.b--point cloud extract*/
    current_frame_.cloud_msg = cloud_msg_queue_.front();
    cloud_msg_queue_.pop_front();
    /*2.c--object detect*/
    object_detect_ptr_->Detect(current_frame_.cloud_msg, current_frame_.objects_msg);
    /*2.d--object track*/

    /*2.e--ground segement*/
    CloudMsg::CLOUD_PTR ground_cloud_ptr(new CloudMsg::CLOUD());
    CloudMsg::CLOUD_PTR no_ground_cloud_ptr(new CloudMsg::CLOUD());
    ground_seg_ptr_->Segement(current_frame_.cloud_msg.cloud_ptr, ground_cloud_ptr, no_ground_cloud_ptr);
    *current_frame_.cloud_msg.cloud_ptr = *no_ground_cloud_ptr;
    /*2.f--imu odom(relative) */
    Eigen::Matrix4f imu_pose = Eigen::Matrix4f::Identity();
    imu_odom_ptr_->ComputeRelativePose(imu_msg_queue_, previous_frame_.time_stamp, current_frame_.time_stamp, imu_pose);
    /*2.g--lidar odom*/
    lidar_odom_ptr_->InitPose(Eigen::Matrix4f::Identity());

    Eigen::Matrix4f corse_pose = Eigen::Matrix4f::Identity();
    lidar_odom_ptr_->ComputeCorsePose(current_frame_.cloud_msg, imu_pose, corse_pose);

    std::cout << "track 1" << std::endl;
    object_track_ptr_->Track(current_frame_.objects_msg, corse_pose);
    std::cout << "track 2" << std::endl;
    CloudMsg::CLOUD_PTR static_cloud_ptr(new CloudMsg::CLOUD());
    CloudMsg::CLOUD_PTR dynamic_cloud_ptr(new CloudMsg::CLOUD());
    DorPost(current_frame_.objects_msg, current_frame_.cloud_msg.cloud_ptr, static_cloud_ptr, dynamic_cloud_ptr);
    *current_frame_.cloud_msg.cloud_ptr = *static_cloud_ptr;

    Eigen::Matrix4f fine_pose = Eigen::Matrix4f::Identity();
    lidar_odom_ptr_->ComputeFinePose(current_frame_.cloud_msg, corse_pose, fine_pose);

    spdlog::info("FrontEnd$ exec {} hz", time_record_ptr_->GetFrequency(1000));

    /*3--display*/
    lidar_odom_pub_ptr_->Publish(fine_pose);
    veh_tf_pub_ptr_->SendTransform(fine_pose);
    CloudMsg::CLOUD_PTR transformed_cloud_ptr(new CloudMsg::CLOUD());
    pcl::transformPointCloud(*current_frame_.cloud_msg.cloud_ptr, *transformed_cloud_ptr, fine_pose);
    pcl::transformPointCloud(*ground_cloud_ptr, *ground_cloud_ptr, fine_pose);
    ground_cloud_pub_ptr_->Publish(ground_cloud_ptr);
    static_cloud_pub_ptr_->Publish(transformed_cloud_ptr);
    current_frame_.objects_msg.TransCoord(fine_pose);
    bbx_pub_ptr_->Publish(current_frame_.objects_msg);

    /*4--update*/
    previous_frame_ = current_frame_;

    return true;

    /*[3]--dynamic removal*/

    /*[5]--display*/
    // lidar_odom_pub_ptr_->Publish(fine_pose);
    // ground_cloud_pub_ptr_->Publish(ground_cloud_ptr);
    // dynamic_cloud_pub_ptr_->Publish(dynamic_cloud_ptr);

    /*[6]--copy to frame*/
    // Frame frame;
    // frame.time_stamp = cloud_msg.time_stamp;                 // timestamp
    // frame.pose = pose;                                       // pose
    // *frame.cloud_msg.cloud_ptr = *cloud_msg.cloud_ptr;       // cloud
    // frame.objects_msg.objects_vec = objects_msg.objects_vec; // ods_vec
    // frame_queue_.push_back(frame);
    //}
    // return true;
}

/**
 * @brief send frame queue to other thread
 * @param[in] frame_queue
 * @param[in] mutex
 * @return
 */
void FrontEndPipe::SendFrameQueue(std::deque<Frame> &frame_queue, std::mutex &mutex)
{
    // if (!frame_queue_.empty())
    // {
    //     for (size_t i = 0; i < frame_queue_.size(); i++)
    //     {
    //         Frame frame = frame_queue_.at(i);

    //         mutex.lock();
    //         frame_queue.push_back(frame);
    //         mutex.unlock();
    //     }
    //     frame_queue_.clear();
    // }
}

/**
 * @brief dynamic object removal
 * @param[in] objects_msg
 * @param[in] cloud_ptr
 * @param[out] static_cloud_ptr
 * @param[out] dynamic_cloud_ptr
 * @return
 */
void FrontEndPipe::DorPost(const ObjectsMsg &objects_msg, const CloudMsg::CLOUD_PTR &cloud_ptr,
                           CloudMsg::CLOUD_PTR &static_cloud_ptr, CloudMsg::CLOUD_PTR &dynamic_cloud_ptr)
{
    std::vector<int> multibox_index;
    multibox_index.reserve(cloud_ptr->points.size());

    for (const auto &object_msg : objects_msg.objects_vec)
    {
        double speed = sqrt(object_msg.v_x * object_msg.v_x + object_msg.v_y * object_msg.v_y);
        if (speed <= 0.2)
        {
            continue;
        }

        pcl::CropBox<CloudMsg::POINT> clipper;

        Eigen::Vector4f minPoint(-object_msg.w * 0.5, -object_msg.l * 0.5, -object_msg.h * 0.5, 1.0);
        Eigen::Vector4f maxPoint(object_msg.w * 0.5, object_msg.l * 0.5, object_msg.h * 0.5, 1.0);

        clipper.setMin(minPoint);
        clipper.setMax(maxPoint);

        clipper.setTranslation(Eigen::Vector3f(object_msg.x, object_msg.y, object_msg.z));
        clipper.setRotation(object_msg.q.matrix().eulerAngles(2, 1, 0));

        clipper.setInputCloud(cloud_ptr);

        std::vector<int> singbox_index;
        clipper.setNegative(false);
        clipper.filter(singbox_index);
        multibox_index.insert(multibox_index.begin(), singbox_index.begin(), singbox_index.end()); //! maybe risk
    }

    pcl::ExtractIndices<CloudMsg::POINT> extract;
    extract.setInputCloud(cloud_ptr);
    extract.setIndices(boost::make_shared<pcl::Indices>(multibox_index));
    extract.setNegative(false); // inline

    static_cloud_ptr.reset(new CloudMsg::CLOUD());
    dynamic_cloud_ptr.reset(new CloudMsg::CLOUD());

    extract.setNegative(true);
    extract.filter(*static_cloud_ptr);

    extract.setNegative(false);
    extract.filter(*dynamic_cloud_ptr);
}

} // namespace evolving_ad_ns
