/**
 * @file    object_track.hpp
 * @brief   3D objection track
 * @author  niu_wengang@163.com
 * @date    2024-05-18
 * @version 0.1.4
 */

#ifndef _OBJECT_TRACK_HPP_
#define _OBJECT_TRACK_HPP_

namespace evolving_ad_ns
{
class ObjectTrack
{
  public:
    ObjectTrack();
    ~ObjectTrack() = default;
    void Track();
};
} // namespace evolving_ad_ns

#endif