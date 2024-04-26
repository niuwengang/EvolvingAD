/**
 * @file    cloud_sub.cpp
 * @brief   里程计订阅
 * @author  niu_wengang@163.com
 * @date    2024-04-06
 * @version 1.0
 */

// user msg lib
#include "user_msg/pose_msg.hpp"
// ros lib
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
// c++
#include <deque>

namespace Tools
{

class OdomSub
{
  public:
    OdomSub() = default;
    OdomSub(ros::NodeHandle &nh, const std::string topic_name, const size_t buffer_size = 1000000);
    ~OdomSub() = default;

    void ParseData(std::deque<PoseMsg> &pose_msg_queue);

  private:
    void MsgCallback(const nav_msgs::OdometryConstPtr &odom_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    std::deque<PoseMsg> pose_msg_queue_;

    std::mutex mutex_;
};

} // namespace Tools