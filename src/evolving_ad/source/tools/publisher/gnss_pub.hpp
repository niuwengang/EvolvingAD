/**
 * @file    gnss_pub.hpp
 * @brief   点云消息发布
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 1.0
 */

#ifndef GNSS_PUB_HPP
#define GNSS_PUB_HPP

// msg lib
#include "user_msg/gnss_msg.hpp"
// ros lib
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

namespace Tools
{

class GnssPub
{
  public:
    GnssPub() = default;
    GnssPub(ros::NodeHandle &nh, const std::string topic_name, const std::string frame_id,
            const size_t buffer_size = 100);
    ~GnssPub() = default;

    void Publish(const GnssMsg &gnss_msg);

  private:
    ros::Publisher pub_;
    std::string frame_id_;
};

} // namespace Tools
#endif