#ifndef _FRAME_HPP_
#define _FRMAE_HPP_

#include "cloud_msg.hpp"

namespace evolving_ad_ns
{

class Frame
{
  public:
    Frame(){};
    Frame(const CloudMsg &in_cloud_msg);
    ~Frame() = default;

    unsigned int index = 0;
    double time_stamp = 0.0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    CloudMsg cloud_msg;
};

} // namespace evolving_ad_ns

#endif //_FRMAE_HPP_