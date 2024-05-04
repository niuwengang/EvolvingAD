#ifndef _FRAME_HPP_
#define _FRAME_HPP_

#include "cloud_msg.hpp"
#include "object_msg.hpp"

namespace evolving_ad_ns
{

class Frame
{
  public:
    Frame() = default;
    Frame &operator=(const Frame &other);
    ~Frame() = default;

    unsigned int index = 0;
    double time_stamp = 0.0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

    CloudMsg cloud_msg;     //! attention deep copy
    ObjectsMsg objects_msg; //! attention difference  ObjectsMsg and ObjectMsg
};

} // namespace evolving_ad_ns

#endif //_FRMAE_HPP_