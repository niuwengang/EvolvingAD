/**
 * @file    cloud_pub.hpp
 * @brief   cloud publisher
 * @author  niu_wengang@163.com
 * @date    2024-04-09
 * @version 0.1.1
 */

#ifndef CLOUD_PUB_HPP_
#define CLOUD_PUB_HPP_

// msg lib
#include "user_msg/cloud_msg.hpp"
// pcl lib
#include <pcl_conversions/pcl_conversions.h>
// ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace Tools
{

class CloudPub
{
  public:
    CloudPub() = default;
    CloudPub(ros::NodeHandle &nh, const std::string topic_name, const std::string frame_id,
             const size_t buffer_size = 10e2);
    ~CloudPub() = default;

    void Publish(const CloudMsg::CLOUD_PTR &cloud_ptr, const double time_stamp);
    void Publish(const CloudMsg &cloud_msg);

  private:
    ros::Publisher pub_;
    std::string frame_id_;
};

} // namespace Tools
#endif // CLOUD_PUB_HPP_