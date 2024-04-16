/**
 * @file    lidar_odom.hpp
 * @brief   lidar odom
 * @author  niu_wengang@163.com
 * @date    2024-04-05
 * @version 0.1.1
 */

#ifndef _LIDAR_ODOM_HPP_
#define _LIDAR_ODOM_HPP_

// eigen
#include <Eigen/Core>
// msg
#include "user_msg/cloud_msg.hpp"
#include "user_msg/frame_msg.hpp"
// pcl
#include <pcl/common/transforms.h>
// module--cloud filter
#include "module/cloud_filter/cloud_filter_interface.hpp"
#include "module/cloud_filter/voxel_filter.hpp"
// module--cloud registration
#include "module/cloud_registration/cloud_registration_interface.hpp"
#include "module/cloud_registration/fast_gicp_registration.hpp"
// yaml
#include <yaml-cpp/yaml.h>

class LidarOdom
{
  public:
    LidarOdom() = delete;
    LidarOdom(const YAML::Node &config_node);
    ~LidarOdom() = default;
    bool InitPose(const Eigen::Matrix4f &lidar_odom_init_pose);              // set inital pose
    bool UpdateOdom(Eigen::Matrix4f &lidar_odom, const CloudMsg &cloud_msg); // update lidar odom

  private:
    bool UpdateLocalMap(const FrameMsg &new_keyframe_msg); // update local map

  private:
    Eigen::Matrix4f lidar_odom_init_pose_ = Eigen::Matrix4f::Identity(); // lidar odom initial pos

    std::shared_ptr<Module::CloudFilterInterface> single_scan_filter_ptr_ = nullptr; // filter for single scan
    std::shared_ptr<Module::CloudFilterInterface> local_map_filter_ptr_ = nullptr;   // filter for local map
    std::shared_ptr<Module::CloudRegistrationInterface> registration_ptr_ = nullptr; // cloud registration

    std::deque<FrameMsg> local_map_keyframe_msg_queue_; // local map frame
    CloudMsg::CLOUD_PTR local_map_ptr_;                 // local map cloud

    struct ParamList
    {
        unsigned int keyframe_num = 10;
        float keyframe_distance = 2.0;
        float single_scan_leaf_size = 1.0;
        float local_map_leaf_size = 0.6;
        float registration_resolution = 1.0;
    } paramlist_;
};

#endif //_LIDAR_ODOM_HPP_