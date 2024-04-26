/**
 * @file    pose_msg.hpp
 * @brief   imu消息封装
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 1.0
 * @note
 * https://docs.ros.org/en/jade/api/sensor_msgs/html/msg/NavSatStatus.html
 */
#ifndef _POSE_MSG_HPP_
#define _POSE_MSG_HPP_

#include <Eigen/Dense>
class PoseMsg
{
  public:
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    double time_stamp = 0.0;
    unsigned int index = 0;
};

#endif