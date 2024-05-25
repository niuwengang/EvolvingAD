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

namespace evolving_ad_ns
{
class ImuOdom
{
  public:
    ImuOdom(const Eigen::Matrix4f &T_lidar2imu);
    // ImuOdom(const YAML::Node &config_node);
    // bool InitPose(const Eigen::Matrix4f &init_pose);
    void ComputeRelativePose(std::deque<ImuMsg> &imu_msg_queue, const double prev_time_stamp,
                             const double curr_time_stamp, Eigen::Matrix4f &relative_pose);

  private:
    bool first_flag_ = true;
    Eigen::Matrix4f T_lidar_imu_ = Eigen::Matrix4f::Identity();
};
} // namespace evolving_ad_ns
#endif
