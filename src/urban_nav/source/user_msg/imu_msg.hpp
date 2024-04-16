/**
 * @file    imu_msg.hpp
 * @brief   imu meesage
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 1.0
 */

#ifndef _IMU_MSG_HPP_
#define _IMU_MSG_HPP_

// msg lib
#include "user_msg/imu_msg.hpp"
// c++ lib
#include <deque>
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

class ImuMsg
{
  public:
    struct LinearAcceleration
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct AngularVelocity
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };
    struct Orientation
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;
    };

  public:
    static bool TimeSync(std::deque<ImuMsg> &unsynced_imu_msg_queue, std::deque<ImuMsg> &synced_imu_msg_queue,
                         const double sync_time);
    Eigen::Matrix3f GetOrientationMatrix();

  public:
    double time_stamp = 0.0;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    Orientation orientation;
};
#endif
