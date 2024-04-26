/**
 * @file    tf_pub.hpp
 * @brief   tf pub
 * @author  niu_wengang@163.com
 * @date    2024-03-30
 * @version 1.0
 */

#ifndef TF_PUB_HPP
#define TF_PUB_HPP

// eigen lib
#include <Eigen/Dense>
// c++ lib
#include <string>
// ros lib
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace Tools
{
class TfPub
{
  public:
    TfPub(const std::string frame_id, const std::string child_frame_id);
    TfPub() = default;
    void SendTransform(const Eigen::Matrix4f pose);

  protected:
    tf::StampedTransform transform_;
    tf::TransformBroadcaster broadcaster_;
};
} // namespace Tools

// namespace Toolsclass TfPub

#endif