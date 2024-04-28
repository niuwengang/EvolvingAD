/**
 * @file    odom_interface.hpp
 * @brief   odom_interface
 * @author  niu_wengang@163.com
 * @date    2024-04-28
 * @version 0.1.1
 */

#ifndef _ODOM_INTERFACE_HPP_
#define _ODOM_INTERFACE_HPP_

#include "msg/frame.hpp"
#include <Eigen/Core>

namespace evolving_ad_ns
{

class OdomInterface
{
  public:
    virtual ~OdomInterface() = default; // must virtual
    virtual bool InitPose(const Eigen::Matrix4f &init_pose) = 0;
    virtual void ComputePose(const NormalFrame &in_normal_frame, Eigen::Matrix4f &new_pose) = 0;
};

} // namespace evolving_ad_ns

#endif