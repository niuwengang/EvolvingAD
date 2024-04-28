#ifndef _FRAME_HPP_
#define _FRMAE_HPP_

#include "cloud_msg.hpp"

namespace evolving_ad_ns
{

class Frame
{
  public:
    double time_stamp = 0.0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    CloudMsg cloud_msg;
};

class KeyFrame : public Frame
{
  public:
    unsigned int index = 0;
    Eigen::Matrix4f opted_pose = Eigen::Matrix4f::Identity();
    CloudMsg transformed_cloud_msg;
};

class NormalFrame : public Frame
{
  public:
    NormalFrame() = default;
    NormalFrame(const CloudMsg &cloud_msg);
};

class DiscardFrame : public Frame
{
  public:
};

} // namespace evolving_ad_ns

#endif //_FRMAE_HPP_