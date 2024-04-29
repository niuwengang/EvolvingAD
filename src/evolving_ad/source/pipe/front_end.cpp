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
    // cloud_pub_ptr_ = std::make_shared<CloudPub>(nh, "front_end_cloud", "map");
    // veh_tf_pub_ptr_ = std::make_shared<TfPub>("map", "ground_link");
    // bbx_pub_ptr_ = std::make_shared<BbxPub>(nh, "front_end_bbx", "map");
    // lidar_odom_pub_ptr_ = std::make_shared<OdomPub>(nh, "front_end_lidar_odom", "map", "lidar");

    /*[4]--algorithm module*/
    object_detect_ptr_ = std::make_shared<ObjectDetect>(paramlist_.model_file_path);
    lidar_odom_ptr_ = std::make_shared<LidarOdom>(config_node["lidar_odom"]);

    /*[4]--tools*/
    time_record_ptr_ = std::make_shared<TimeRecord>();

    spdlog::info("FrontEnd$ init success");
    std::thread::id threadId = std::this_thread::get_id();
    spdlog::info("FrontEnd$ thread id:{}", *(std::thread::native_handle_type *)(&threadId));
}

bool FrontEndPipe::Run()
{

    cloud_sub_ptr_->ParseMsg(cloud_msg_queue_);

    if (!cloud_msg_queue_.empty())
    {
        CloudMsg cloud_msg = cloud_msg_queue_.front();
        cloud_msg_queue_.pop_front();
        ObjectsMsg objects_msg;

        time_record_ptr_->Start();

        object_detect_ptr_->Detect(cloud_msg, objects_msg);
        lidar_odom_ptr_->InitPose(Eigen::Matrix4f::Identity());
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        lidar_odom_ptr_->ComputePose(cloud_msg, pose);

        spdlog::info("Front$ exec {} hz", time_record_ptr_->GetFrequency(1000));

        // bbx_pub_ptr_->Publish(objects_msg);
        // cloud_pub_ptr_->Publish(cloud_msg);
        // veh_tf_pub_ptr_->SendTransform(pose);
        // lidar_odom_pub_ptr_->Publish(pose, 0);

        Frame frame;
        frame.time_stamp = cloud_msg.time_stamp;
        frame.pose = pose;
        *frame.cloud_msg.cloud_ptr = *cloud_msg.cloud_ptr; //! deep copy
        frame_queue_.push_back(frame);
    }

    return true;
}

void FrontEndPipe::SendFrameQueue(std::deque<Frame> &frame_queue, std::mutex &mutex)
{

    if (!frame_queue_.empty())
    {
        for (int i = 0; i < frame_queue_.size(); i++)
        {
            Frame frame; // deep copy
            frame.index = frame_queue_.at(i).index;
            frame.time_stamp = frame_queue_.at(i).time_stamp;
            frame.pose = frame_queue_.at(i).pose;
            *frame.cloud_msg.cloud_ptr = *frame_queue_.at(i).cloud_msg.cloud_ptr;

            mutex.lock();
            frame_queue.push_back(frame);
            mutex.unlock();
        }
        frame_queue_.clear();
    }
}

} // namespace evolving_ad_ns
