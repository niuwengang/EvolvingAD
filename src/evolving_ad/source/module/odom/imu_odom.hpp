/**
 * @file  imu_odom.hpp
 * @brief imu odom
 * @author  niu_wengang@163.com
 * @date    2024-05-23
 * @version 0.0
 */

#ifndef _IMU_ODOM_HPP_
#define _IMU_ODOM_HPP_

// yaml
#include "msg/imu_msg.hpp"
#include <yaml-cpp/yaml.h>

class ImuOdom
{
  public:
    ImuOdom(const YAML::Node &config_node);
    // bool ComputeRelativePose(std::deque<ImuMsg> imu_msg_queue);

  private:
};

#endif
