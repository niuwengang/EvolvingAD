/**
 * @file    imu_pub.hpp
 * @brief   imu消息发布
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 1.0
 */

#ifndef IMU_PUB_HPP
#define IMU_PUB_HPP

// user msg
#include "user_msg/imu_msg.hpp"
// c++ lib
#include <deque>
// ros lib
#include "sensor_msgs/Imu.h"
#include <ros/ros.h>

namespace Tools
{

class ImuPub
{
  public:
    ImuPub() = default;
    ImuPub(ros::NodeHandle &nh, const std::string topic_name, const std::string frame_id, const size_t buffer_size = 1);
    ~ImuPub() = default;

    void Publish(const ImuMsg &imu_msg);

  private:
    ros::Publisher pub_;
    std::string frame_id_;
};

} // namespace Tools
#endif // IMU_PUB_HPP