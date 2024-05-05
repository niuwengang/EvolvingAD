#ifndef _GNSS_ODOM_HPP_
#define _GNSS_ODOM_HPP_

// eigen
#include <Eigen/Core>
// geograph
#include <GeographicLib/LocalCartesian.hpp>
// c++
#include <deque>
#include <iostream>
// yaml
#include <yaml-cpp/yaml.h>
// msg
#include "msg/gnss_msg.hpp"

namespace evolving_ad_ns
{

class GnssOdom
{
  public:
    GnssOdom(const YAML::Node &config_node);
    bool InitPose(GnssMsg &gnss_msg);
    void ComputePose(GnssMsg &gnss_msg, Eigen::Matrix4f &new_pose);

  private:
    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
    bool init_flag_ = false;
};

} // namespace evolving_ad_ns
#endif //_GNSS_MSG_HPP_