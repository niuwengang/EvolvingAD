#ifndef _FRAME_HPP_
#define _FRAME_HPP_

#include "cloud_msg.hpp"
#include "object_msg.hpp"

namespace evolving_ad_ns
{

class Frame
{
  public:
    Frame();
    Frame(const Frame &other);
    Frame &operator=(const Frame &other);
    ~Frame();

    unsigned int index = 0;
    double time_stamp = 0.0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

    CloudMsg cloud_msg;
    ObjectsMsg objects_msg;
};

} // namespace evolving_ad_ns

#endif //_FRMAE_HPP_