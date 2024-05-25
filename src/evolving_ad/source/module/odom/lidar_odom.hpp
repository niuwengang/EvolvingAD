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
//
#include <execution>

namespace evolving_ad_ns
{
class LidarOdom
{
  public:
    LidarOdom(const YAML::Node &config_node);
    bool InitPose(const Eigen::Matrix4f &init_pose);
    void ComputeCorsePose(const CloudMsg &cloud_msg, const Eigen::Matrix4f &imu_pose, Eigen::Matrix4f &corse_pose);
    void ComputeFinePose(const CloudMsg &cloud_msg, const Eigen::Matrix4f &corse_pose, Eigen::Matrix4f &fine_pose);

  private:
    // void UpdateLocalMap(const Frame &normal_frame);

    std::shared_ptr<CloudFilterInterface> filter_small_size_ptr_ = nullptr;  // filter for single scan
    std::shared_ptr<CloudFilterInterface> filter_media_size_ptr_ = nullptr;  // filter for local map
    std::shared_ptr<CloudFilterInterface> filter_large_size_ptr_ = nullptr;  // filter for local map
    std::shared_ptr<CloudRegistrationInterface> registration_ptr_ = nullptr; // cloud registration

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f last_keyframe_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f predict_pose_ = Eigen::Matrix4f::Identity();
    std::deque<Frame> local_keyframe_queue_;
    CloudMsg::CLOUD_PTR local_map_ptr_;
    CloudMsg cloud_msg_pre_;
    bool init_flag_ = false;
    bool first_flag_ = true;

    struct ParamList
    {
        unsigned int keyframe_num = 5;
        float keyframe_distance = 2.0;

        float filter_leaf_size_small = 0.0;
        float filter_leaf_size_media = 0.0;
        float filter_leaf_size_large = 0.0;

        float registration_resolution = 0.0;
    } paramlist_;
};

} // namespace evolving_ad_ns

#endif //_LIDAR_ODOM_HPP_