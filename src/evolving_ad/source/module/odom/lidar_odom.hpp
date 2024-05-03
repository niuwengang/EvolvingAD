/**
 * @file
 * @brief
 * @author  niu_wengang@163.com
 * @date    2024-03-28
 * @version 1.0
 */

#ifndef _LIDAR_ODOM_HPP_
#define _LIDAR_ODOM_HPP_

// yaml
#include <yaml-cpp/yaml.h>
// module--cloud filter
#include "module/cloud_filter/cloud_filter_interface.hpp"
#include "module/cloud_filter/voxel_filter.hpp"
// module--cloud registration
#include "module/cloud_registration/cloud_registration_interface.hpp"
#include "module/cloud_registration/fast_gicp_registration.hpp"
// msg
#include "msg/frame.hpp"

namespace evolving_ad_ns
{
class LidarOdom
{
  public:
    LidarOdom(const YAML::Node &config_node);
    bool InitPose(const Eigen::Matrix4f &init_pose);
    void ComputePose(const CloudMsg &cloud_msg, Eigen::Matrix4f &new_pose);

  private:
    void UpdateLocalMap(const Frame &normal_frame);

    std::shared_ptr<CloudFilterInterface> single_scan_filter_ptr_ = nullptr; // filter for single scan
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_ = nullptr;   // filter for local map
    std::shared_ptr<CloudRegistrationInterface> registration_ptr_ = nullptr; // cloud registration

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();

    std::deque<Frame> local_keyframe_queue_;
    CloudMsg::CLOUD_PTR local_map_ptr_;

    struct ParamList
    {
        unsigned int keyframe_num = 10;
        float keyframe_distance = 2.0;
        float single_scan_leaf_size = 1.0;
        float local_map_leaf_size = 0.6;
        float registration_resolution = 1.0;
    } paramlist_;
};

} // namespace evolving_ad_ns

#endif //_LIDAR_ODOM_HPP_