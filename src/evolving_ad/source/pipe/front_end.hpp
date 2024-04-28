/**
 * @file    front_end_pipe.hpp
 * @brief   loc front end pipeline
 * @author  niu_wengang@163.com
 * @date    2024-04-28
 * @version 0.1.4
 */

#ifndef _FRONT_END_HPP_
#define _FRONT_END_HPP_

// yaml
#include <yaml-cpp/yaml.h>
// ros
#include <ros/ros.h>
// topic sub
#include "topic_sub/cloud_sub.hpp"
// topic pub
#include "topic_pub/bbx_pub.hpp"
#include "topic_pub/cloud_pub.hpp"
#include "topic_pub/odom_pub.hpp"
#include "topic_pub/tf_pub.hpp"
// msg
#include "msg/object_msg.hpp"
// module
#include "module/object_detect/object_detect.hpp"
#include "module/odom/lidar_odom.hpp"
#include "module/odom/odom_interface.hpp"

namespace evolving_ad_ns
{
class FrontEndPipe
{
  public:
    FrontEndPipe() = delete;
    FrontEndPipe(ros::NodeHandle &nh, const std::string package_folder_path);
    ~FrontEndPipe() = default;

    bool Run();

  private:
    /*pub and sub*/
    std::shared_ptr<CloudSub> cloud_sub_ptr_ = nullptr;
    std::shared_ptr<CloudPub> cloud_pub_ptr_ = nullptr;
    std::shared_ptr<TfPub> veh_tf_pub_ptr_ = nullptr;
    std::shared_ptr<BbxPub> bbx_pub_ptr_ = nullptr;
    std::shared_ptr<OdomPub> odom_pub_ptr_ = nullptr;

    /*variable*/
    std::deque<CloudMsg> cloud_msg_queue_;

    /*algorithm module*/
    std::shared_ptr<ObjectDetect> object_detect_ptr_ = nullptr;
    std::shared_ptr<OdomInterface> lidar_odom_ptr_ = nullptr;

    struct ParamList
    {
      public:
        std::string package_folder_path;
        std::string model_file_path;
        std::string cloud_sub_topic;
        std::string cloud_pub_topic;
        std::string odom_pub_topic;

    } paramlist_;
};
} // namespace evolving_ad_ns

#endif //_FRONT_END_HPP_