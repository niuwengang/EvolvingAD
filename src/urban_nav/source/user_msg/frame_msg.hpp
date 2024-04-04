#ifndef FRAME_MSG_HPP
#define FRAME_MSG_HPP

#include "cloud_msg.hpp"
// eigen lib
#include <Eigen/Core>

class Frame
{
  public:
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    CloudMsg cloud_msg;
};

#endif