/**
 * @file    object_track.hpp
 * @brief   3D objection track
 * @author  niu_wengang@163.com
 * @date    2024-05-18
 * @version 0.1.4
 */

#ifndef _OBJECT_TRACK_HPP_
#define _OBJECT_TRACK_HPP_

#include "thirdpartylib/hungarian/include/Hungarian.h"

// msg
#include "msg/object_msg.hpp"

namespace evolving_ad_ns
{
class ObjectTrack
{
  public:
    ObjectTrack();
    ~ObjectTrack();
    void Track(ObjectsMsg &objects_msg_curr, const Eigen::Matrix4f &relative_pose);

  private:
    bool first_flag_ = true;
    const unsigned int max_id = 4294967295;
    unsigned int id_ = 0; // 4294967295
    ObjectsMsg objects_msg_esti_;
};
} // namespace evolving_ad_ns

#endif