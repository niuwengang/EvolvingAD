/**
 * @file    imu_msg.cpp
 * @brief   imu msg
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 1.0
 */

#include "imu_msg.hpp"

namespace evolving_ad_ns
{
Eigen::Matrix3f ImuMsg::GetOrientationMatrix()
{
    Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
    Eigen::Matrix3f matrix = q.matrix().cast<float>();

    return matrix;
}
} // namespace evolving_ad_ns
