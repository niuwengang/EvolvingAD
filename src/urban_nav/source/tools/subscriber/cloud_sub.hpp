/**
 * @file    cloud_sub.hpp
 * @brief   subscribe cloud
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 0.1.1
 */

#ifndef _CLOUD_SUB_HPP_
#define _CLOUD_SUB_HPP_

// user msg
#include "user_msg/cloud_msg.hpp"
// c++
#include <deque>
#include <mutex>
// pcl
#include <pcl_conversions/pcl_conversions.h>
// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace Tools
{

class CloudSub
{
  public:
    CloudSub() = delete;
    CloudSub(ros::NodeHandle &nh, const std::string topic_name, const size_t buffer_size = 10e5);
    ~CloudSub() = default;
    void ParseData(std::deque<CloudMsg> &cloud_msg_queue);

  private:
    void MsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::mutex mutex_;

    std::deque<CloudMsg> cloud_msg_queue_;
};
} // namespace Tools
#endif //_CLOUD_SUB_HPP_