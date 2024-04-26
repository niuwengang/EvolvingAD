/**
 * @file    imu_pub.hpp
 * @brief   publish imu
 * @author  niu_wengang@163.com
 * @date    2024-04-09
 * @version 0.1.1
 */

#ifndef _IMU_PUB_HPP_
#define _IMU_PUB_HPP_

// user msg
#include "user_msg/imu_msg.hpp"
// c++
#include <deque>
// ros
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

namespace Tools
{

class ImuPub
{
  public:
    ImuPub() = delete;
    ImuPub(ros::NodeHandle &nh, const std::string topic_name, const std::string frame_id,
           const size_t buffer_size = 10e2);
    ~ImuPub() = default;
    void Publish(const ImuMsg &imu_msg);

  private:
    ros::Publisher pub_;
    std::string frame_id_;
};

} // namespace Tools
#endif // _IMU_PUB_HPP_