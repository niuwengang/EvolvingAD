/**
 * @file    tf_pub.hpp
 * @brief   tf pub
 * @author  niu_wengang@163.com
 * @date    2024-03-30
 * @version 0.1.0
 */

#ifndef _TF_PUB_HPP_
#define _TF_PUB_HPP_

// eigen
#include <Eigen/Dense>
// c++
#include <string>
// ros
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace evolving_ad_ns
{
class TfPub
{
  public:
    TfPub(const std::string frame_id, const std::string child_frame_id);
    TfPub() = default;
    void SendTransform(const Eigen::Matrix4f pose);

  private:
    tf::StampedTransform transform_;
    tf::TransformBroadcaster broadcaster_;
};
} // namespace evolving_ad_ns

// namespace Toolsclass TfPub

#endif