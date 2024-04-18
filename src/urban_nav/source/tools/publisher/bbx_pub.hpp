#ifndef _BBX_PUB_HPP_
#define _BBX_PUB_HPP_

/**
 * @file    bbx_pub.hpp
 * @brief   publish imu
 * @author  niu_wengang@163.com
 * @date    2024-04-12
 * @version 0.1.1
 */

// ros
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include "user_msg/ods_msg.hpp"

namespace Tools
{

class BbxPub
{
  public:
    BbxPub() = delete;
    BbxPub(ros::NodeHandle &nh, const std::string topic_name, const std::string frame_id,
           const size_t buffer_size = 10e2);
    ~BbxPub() = default;

    void Publish(const OdsMsg &ods_msg);

  private:
    ros::Publisher pub_;
    std::string frame_id_;
};

} // namespace Tools
#endif // _BOX_HPP_