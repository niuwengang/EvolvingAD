#ifndef _PATH_PUB_HPP_
#define _PATH_PUB_HPP_

#include "path_pub.hpp"
// eigen
#include <Eigen/Dense>
// c++
#include <deque>
#include <string>
// ros
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

namespace Tools
{

class PathPub
{
  public:
    PathPub(ros::NodeHandle &nh, const std::string topic_name, const std::string frame_id,
            const size_t buffer_size = 10e2);
    PathPub() = default;

    void Publish(const std::deque<Eigen::Matrix4f> &pose_queue, const double time_stamp = 0);

  private:
    ros::Publisher pub_;
    std::string frame_id_ = "";
};
} // namespace Tools

#endif