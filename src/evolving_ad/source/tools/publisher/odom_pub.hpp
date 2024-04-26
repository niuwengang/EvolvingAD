/**
 * @file    odom_pub.hpp
 * @brief   publish odom
 * @author  niu_wengang@163.com
 * @date    2024-04-09
 * @version 0.1.1
 */

#ifndef _ODOM_PUB_HPP_
#define _ODOM_PUB_HPP_

// eigen
#include <Eigen/Dense>
// ros
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
// c++
#include <string>

namespace Tools
{

class OdomPub
{
  public:
    OdomPub() = delete;
    OdomPub(ros::NodeHandle &nh, const std::string topic_name, const std::string base_frame_id,
            const std::string child_frame_id, const size_t buffer_size = 10e2);
    ~OdomPub() = default;

    void Publish(const Eigen::Matrix4f &transform_matrix, const double time_stamp);

  private:
    ros::Publisher pub_;
    nav_msgs::Odometry odom_;
};
} // namespace Tools

#endif // ODOM_PUB_HPP