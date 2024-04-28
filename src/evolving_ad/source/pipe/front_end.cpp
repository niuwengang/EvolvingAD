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

    YAML::Node config_node = YAML::LoadFile(paramlist_.package_folder_path + "/config/front_end.yaml");

    paramlist_.cloud_sub_topic = config_node["topic_sub"]["cloud_sub_topic"].as<std::string>();
    paramlist_.cloud_pub_topic = config_node["topic_pub"]["cloud_pub_topic"].as<std::string>();
    paramlist_.odom_pub_topic = config_node["topic_pub"]["odom_pub_topic"].as<std::string>();

    paramlist_.model_file_path = paramlist_.package_folder_path + "/model//pointpillar.onnx";
    /*[2]--topic sub and pub*/
    cloud_sub_ptr_ = std::make_shared<CloudSub>(nh, paramlist_.cloud_sub_topic);
    cloud_pub_ptr_ = std::make_shared<CloudPub>(nh, paramlist_.cloud_pub_topic, "map");
    veh_tf_pub_ptr_ = std::make_shared<TfPub>("map", "ground_link");
    bbx_pub_ptr_ = std::make_shared<BbxPub>(nh, "object_result", "map");
    odom_pub_ptr_ = std::make_shared<OdomPub>(nh, "my_odom", "map", "lidar");

    /*algorithm module*/
    object_detect_ptr_ = std::make_shared<ObjectDetect>(paramlist_.model_file_path);
    lidar_odom_ptr_ = std::make_shared<LidarOdom>(config_node["lidar_odom"]);
}

bool FrontEndPipe::Run()
{

    cloud_sub_ptr_->ParseMsg(cloud_msg_queue_);

    if (!cloud_msg_queue_.empty())
    {
        CloudMsg cloud_msg = cloud_msg_queue_.front();
        cloud_msg_queue_.pop_front();

        ObjectsMsg objects_msg;
        object_detect_ptr_->Detect(cloud_msg, objects_msg);

        lidar_odom_ptr_->InitPose(Eigen::Matrix4f::Identity());

        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        NormalFrame normal_frame(cloud_msg);
        lidar_odom_ptr_->ComputePose(normal_frame, pose);

        bbx_pub_ptr_->Publish(objects_msg);
        cloud_pub_ptr_->Publish(cloud_msg);
        veh_tf_pub_ptr_->SendTransform(pose);
        odom_pub_ptr_->Publish(pose, cloud_msg.time_stamp);
    }

    return true;
}

} // namespace evolving_ad_ns
