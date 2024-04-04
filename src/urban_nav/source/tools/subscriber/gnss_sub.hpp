/**
 * @file    gnss_sub.hpp
 * @brief   gnss消息订阅
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 1.0
 * @note
 * https://docs.ros.org/en/jade/api/sensor_msgs/html/msg/NavSatFix.html
 */

// c++
#include <deque>
#include <mutex>
// msg
#include "user_msg/gnss_msg.hpp"
// ros lib
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

namespace Tools
{

class GnssSub
{
  public:
    GnssSub() = default;
    GnssSub(ros::NodeHandle &nh, const std::string topic_name, const size_t buffer_size = 1000000);
    ~GnssSub() = default;
    void ParseData(std::deque<GnssMsg> &gnss_msg_queue);

  private:
    std::deque<GnssMsg> gnss_msg_queue_;
    ros::Subscriber sub_;
    std::mutex mutex_;

  private:
    void MsgCallback(const sensor_msgs::NavSatFixConstPtr &gnss_msg_ptr);
};
} // namespace Tools