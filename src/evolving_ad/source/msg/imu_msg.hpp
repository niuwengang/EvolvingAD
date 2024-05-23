/**
 * @file    imu_msg.hpp
 * @brief   imu msg
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 1.0
 */

#ifndef _IMU_SUB_HPP_
#define _IMU_SUB_HPP_

// msg lib
#include "imu_msg.hpp"
// c++ lib
#include <deque>
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace evolving_ad_ns
{
class ImuMsg
{
  public:
    struct LinearAcceleration
    {
        double x = 0.0, y = 0.0, z = 0.0;
    };

    struct AngularVelocity
    {
        double x = 0.0, y = 0.0, z = 0.0;
    };
    struct Orientation
    {
        double x = 0.0, y = 0.0, z = 0.0, w = 0.0;
    };

  public:
    Eigen::Matrix3f GetOrientationMatrix();

  public:
    double time_stamp = 0.0;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    Orientation orientation;
};
} // namespace evolving_ad_ns

#endif
