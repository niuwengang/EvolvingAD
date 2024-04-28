/**
 * @file    front_end_pipe.cpp
 * @brief   loc front end pipeline
 * @author  niu_wengang@163.com
 * @date    2024-04-28
 * @version 0.1.4
 */

#include "front_end.hpp"
// yaml
#include <yaml-cpp/yaml.h>

namespace evolving_ad_ns
{
FrontEndPipe::FrontEndPipe(ros::NodeHandle &nh, const std::string package_folder_path)
{
    /*[1]--load params*/
    paramlist_.package_folder_path = package_folder_path;

    YAML::Node config_node = YAML::LoadFile(paramlist_.package_folder_path + "/config/front_end.yaml");

    paramlist_.cloud_sub_topic = config_node["topic_sub"]["cloud_sub_topic"].as<std::string>();
    paramlist_.cloud_pub_topic = config_node["topic_pub"]["cloud_pub_topic"].as<std::string>();

    std::cout << "cloud_sub_topic path:" << paramlist_.cloud_sub_topic << std::endl;
    cloud_sub_ptr_ = std::make_shared<CloudSub>(nh, paramlist_.cloud_sub_topic);
    cloud_pub_ptr_ = std::make_shared<CloudPub>(nh, paramlist_.cloud_pub_topic, "map");
    // gnss sub

    /*[2]--topic pu*/
    // lidar pub
}

bool FrontEndPipe::Run()
{
    cloud_sub_ptr_->ParseMsg(cloud_msg_queue_);

    if (!cloud_msg_queue_.empty())
    {
        CloudMsg cloud_msg = cloud_msg_queue_.front();
        cloud_msg_queue_.pop_front();
        cloud_pub_ptr_->Publish(cloud_msg);
    }

    return true;
}

} // namespace evolving_ad_ns
