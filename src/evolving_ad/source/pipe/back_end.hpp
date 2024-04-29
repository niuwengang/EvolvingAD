#ifndef _BACK_END_HPP_
#define _BACK_END_HPP_

// yaml
#include <yaml-cpp/yaml.h>
// ros
#include <ros/ros.h>

namespace evolving_ad_ns
{

class BackEndPipe
{
  public:
    BackEndPipe() = delete;
    BackEndPipe(ros::NodeHandle &nh, const std::string package_folder_path);
    ~BackEndPipe() = default;
    bool Run();
};
} // namespace evolving_ad_ns
#endif