/**
 * @file    bbx_pub.hpp
 * @brief   publish imu
 * @author  niu_wengang@163.com
 * @date    2024-04-12
 * @version 0.1.1
 */

#ifndef _BBX_PUB_HPP_
#define _BBX_PUB_HPP_

// ros
#include "msg/object_msg.hpp"
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace evolving_ad_ns
{

class BbxPub
{
  public:
    BbxPub() = delete;
    BbxPub(ros::NodeHandle &nh, const std::string topic_name, const std::string frame_id,
           const size_t buffer_size = 10);
    ~BbxPub() = default;

    void Publish(const ObjectsMsg &objects_msg);

  private:
    ros::Publisher bbx_pub_;
    ros::Publisher marker_pub_;
    std::string frame_id_;
};

} // namespace evolving_ad_ns
#endif // _BOX_HPP_