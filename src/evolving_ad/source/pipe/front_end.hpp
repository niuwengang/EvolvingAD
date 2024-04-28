/**
 * @file    front_end_pipe.hpp
 * @brief   loc front end pipeline
 * @author  niu_wengang@163.com
 * @date    2024-04-28
 * @version 0.1.4
 */

#ifndef _FRONT_END_HPP_
#define _FRONT_END_HPP_

// ros
#include <ros/ros.h>
// topic sub
#include "topic_sub/cloud_sub.hpp"
// topic pub
#include "topic_pub/cloud_pub.hpp"

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

    /*variable*/
    std::deque<CloudMsg> cloud_msg_queue_;

    struct ParamList
    {
      public:
        std::string package_folder_path;
        std::string cloud_sub_topic;
        std::string cloud_pub_topic;
    } paramlist_;
};
} // namespace evolving_ad_ns

#endif //_FRONT_END_HPP_