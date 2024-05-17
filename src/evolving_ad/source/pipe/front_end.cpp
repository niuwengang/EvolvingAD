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
    /*[1]--load params*/
    paramlist_.package_folder_path = package_folder_path;
    YAML::Node config_node = YAML::LoadFile(paramlist_.package_folder_path + "/config/ad.yaml");
    paramlist_.cloud_sub_topic = config_node["topic_sub"]["cloud_sub_topic"].as<std::string>();
    paramlist_.model_file_path = paramlist_.package_folder_path + "/model//pointpillar.onnx";

    /*[2]--topic sub */
    cloud_sub_ptr_ = std::make_shared<CloudSub>(nh, paramlist_.cloud_sub_topic);

    /*[3]--topic pub */
    static_cloud_pub_ptr_ = std::make_shared<CloudPub>(nh, "static_cloud", "map");
    ground_cloud_pub_ptr_ = std::make_shared<CloudPub>(nh, "ground_cloud", "map");
    dynamic_cloud_pub_ptr_ = std::make_shared<CloudPub>(nh, "dynamic_cloud", "map");
    bbx_pub_ptr_ = std::make_shared<BbxPub>(nh, "bbx", "map");

    /*[4]--algorithm module*/
    object_detect_ptr_ = std::make_shared<ObjectDetect>(paramlist_.model_file_path);
    lidar_odom_ptr_ = std::make_shared<LidarOdom>(config_node["lidar_odom"]);
    ground_seg_ptr_ = std::make_shared<DipgGroundSegment>();

    /*[5]--tools*/
    time_record_ptr_ = std::make_shared<TimeRecord>();

    /*[6]--log*/
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
    cloud_sub_ptr_->ParseMsg(cloud_msg_queue_);

    if (!cloud_msg_queue_.empty())
    {
        CloudMsg cloud_msg = cloud_msg_queue_.front();
        cloud_msg_queue_.pop_front();

        time_record_ptr_->Start();

        std::vector<int> indices;
        CloudMsg::CLOUD_PTR nan_cloud_ptr(new CloudMsg::CLOUD());
        pcl::removeNaNFromPointCloud(*cloud_msg.cloud_ptr, *nan_cloud_ptr, indices);
        *cloud_msg.cloud_ptr = *nan_cloud_ptr;

        /*[1]--object detection*/
        ObjectsMsg objects_msg;
        object_detect_ptr_->Detect(cloud_msg, objects_msg);

        /*[2]--ground segement*/
        // CloudMsg::CLOUD_PTR ground_cloud_ptr(new CloudMsg::CLOUD());
        // CloudMsg::CLOUD_PTR no_ground_cloud_ptr(new CloudMsg::CLOUD());
        // ground_seg_ptr_->Segement(cloud_msg.cloud_ptr, ground_cloud_ptr, no_ground_cloud_ptr);

        /*[3]--dynamic removal*/
        // CloudMsg::CLOUD_PTR static_cloud_ptr(new CloudMsg::CLOUD());
        // CloudMsg::CLOUD_PTR dynamic_cloud_ptr(new CloudMsg::CLOUD());
        // DorPost(objects_msg, no_ground_cloud_ptr_, static_cloud_ptr, dynamic_cloud_ptr);
        // *cloud_msg.cloud_ptr = *static_cloud_ptr;

        /*[4]--lidar odom*/
        // lidar_odom_ptr_->InitPose(Eigen::Matrix4f::Identity());
        // Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        // lidar_odom_ptr_->ComputePose(cloud_msg, pose);

        spdlog::info("FrontEnd$ exec {} hz", time_record_ptr_->GetFrequency(1000));

        /*[5]--display*/
        bbx_pub_ptr_->Publish(objects_msg);
        static_cloud_pub_ptr_->Publish(cloud_msg);
        // ground_cloud_pub_ptr_->Publish(ground_cloud_ptr);
        // dynamic_cloud_pub_ptr_->Publish(dynamic_cloud_ptr);

        /*[6]--copy to frame*/
        // Frame frame;
        // frame.time_stamp = cloud_msg.time_stamp;                 // timestamp
        // frame.pose = pose;                                       // pose
        // *frame.cloud_msg.cloud_ptr = *cloud_msg.cloud_ptr;       // cloud
        // frame.objects_msg.objects_vec = objects_msg.objects_vec; // ods_vec
        // frame_queue_.push_back(frame);
    }
    return true;
}

/**
 * @brief send frame queue to other thread
 * @param[in] frame_queue
 * @param[in] mutex
 * @return
 */
void FrontEndPipe::SendFrameQueue(std::deque<Frame> &frame_queue, std::mutex &mutex)
{
    if (!frame_queue_.empty())
    {
        for (size_t i = 0; i < frame_queue_.size(); i++)
        {
            Frame frame = frame_queue_.at(i);

            mutex.lock();
            frame_queue.push_back(frame);
            mutex.unlock();
        }
        frame_queue_.clear();
    }
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
