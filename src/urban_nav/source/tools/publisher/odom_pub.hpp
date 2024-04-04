/**
 * @file    odom_pub.hpp
 * @brief   里程计发布
 * @author  niu_wengang@163.com
 * @date    2024-03-30
 * @version 1.0
 */

#ifndef ODOM_PUB_HPP
#define ODOM_PUB_HPP

// eigen
#include <Eigen/Dense>
// ros lib
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
// c++ lib
#include <string>

namespace Tools
{

class OdomPub
{
  public:
    OdomPub() = default;
    OdomPub(ros::NodeHandle &nh, const std::string topic_name, const std::string base_frame_id,
            const std::string child_frame_id, const size_t buffer_size = 1);
    ~OdomPub() = default;

    void Publish(const Eigen::Matrix4f &transform_matrix);

  private:
    ros::Publisher pub_;
    nav_msgs::Odometry odom_;
};
} // namespace Tools

#endif // ODOM_PUB_HPP