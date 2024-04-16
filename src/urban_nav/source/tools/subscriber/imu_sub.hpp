/**
 * @file    imu_sub.hpp
 * @brief   subscribe imu
 * @author  niu_wengang@163.com
 * @date    2024-04-09 update
 * @version 0.1.1
 */
#ifndef _IMU_SUB_HPP_
#define _IMU_SUB_HPP_

// user msg
#include "user_msg/imu_msg.hpp"
// c++
#include <deque>
#include <mutex>
// ros
#include "sensor_msgs/Imu.h"
#include <ros/ros.h>

namespace Tools
{
class ImuSub
{
  public:
    ImuSub() = delete;
    ImuSub(ros::NodeHandle &nh, const std::string topic_name, const size_t buffer_size = 10e5);
    ~ImuSub() = default;
    void ParseData(std::deque<ImuMsg> &imu_msg_queue);

  private:
    void MsgCallback(const sensor_msgs::ImuConstPtr &imu_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    std::deque<ImuMsg> imu_msg_queue_;

    std::mutex mutex_;
};
} // namespace Tools

#endif //_IMU_SUB_HPP_