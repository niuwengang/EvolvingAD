#ifndef _FRAME_MSG_HPP_
#define _FRAME_MSG_HPP_

// user msg
#include "cloud_msg.hpp"
// eigen lib
#include <Eigen/Core>

class FrameMsg
{
  public:
    double time_stamp = 0.0;
    unsigned int index = 0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    CloudMsg cloud_msg;
    // bool is_keyframe = true;//
};

#endif //_FRAME_MSG_HPP_