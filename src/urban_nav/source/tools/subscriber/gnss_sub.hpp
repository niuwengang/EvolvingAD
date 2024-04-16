/**
 * @file    gnss_sub.hpp
 * @brief   gnss subscriber
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 0.1.1
 * @note
 * https://docs.ros.org/en/jade/api/sensor_msgs/html/msg/NavSatFix.html
 */

#ifndef _GNSS_SUB_HPP_
#define _GNSS_SUB_HPP_

// c++
#include <deque>
#include <mutex>
// msg
#include "user_msg/gnss_msg.hpp"
// ros
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

namespace Tools
{

class GnssSub
{
  public:
    GnssSub() = delete;
    GnssSub(ros::NodeHandle &nh, const std::string topic_name, const size_t buffer_size = 10e5);
    ~GnssSub() = default;
    void ParseData(std::deque<GnssMsg> &gnss_msg_queue);

  private:
    void MsgCallback(const sensor_msgs::NavSatFixConstPtr &gnss_msg_ptr);

  private:
    std::deque<GnssMsg> gnss_msg_queue_;
    ros::Subscriber sub_;
    std::mutex mutex_;
};
} // namespace Tools

#endif //_GNSS_SUB_HPP_