#ifndef _GNSS_ODOM_HPP_
#define _GNSS_ODOM_HPP_

/**
 * @file    gnss_odom.hpp
 * @brief   gnss odom calculate
 * @author  niu_wengang@163.com
 * @date    2024-04-05
 * @version 0.1.1
 */

// eigen
#include <Eigen/Core>
// user msg
#include "user_msg/gnss_msg.hpp"
#include "user_msg/imu_msg.hpp"

namespace Module
{

class GnssOdom
{
  public:
    GnssOdom();
    ~GnssOdom() = default;
    bool InitPose(GnssMsg &gnss_msg);                                                // set ENU pos
    bool UpdateOdom(Eigen::Matrix4f &gnss_odom, GnssMsg &gnss_msg, ImuMsg &imu_msg); // 更新里程计
  private:
    bool gnss_odom_init_flag_ = false; // 初始化标志位
};

} // namespace Module
#endif //_GNSS_ODOM_HPP_