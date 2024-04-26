/**
 * @file    lidar_odom.hpp
 * @brief   lidar odom
 * @author  niu_wengang@163.com
 * @date    2024-04-05
 * @version 0.1.1
 */
#include "lidar_odom.hpp"

/**
 * @brief set params
 * @param[in] keyframe_num
 * @param[in] keyframe_distance
 * @param[in] single_scan_leaf_size filter size
 * @param[in] local_map_leaf_size filter size
 * @return
 */
LidarOdom::LidarOdom(const YAML::Node &config_node)
{
    /*[1]--reset value*/
    lidar_odom_init_pose_ = Eigen::Matrix4f::Identity();
    local_map_ptr_.reset(new CloudMsg::CLOUD());

    /*[2]--fill paramlist*/
    paramlist_.keyframe_distance = config_node["keyframe_distance"].as<float>();
    paramlist_.keyframe_num = config_node["keyframe_num"].as<int>();
    paramlist_.single_scan_leaf_size = config_node["single_scan_leaf_size"].as<float>();
    paramlist_.local_map_leaf_size = config_node["local_map_leaf_size"].as<float>();
    paramlist_.registration_resolution = config_node["registration_resolution"].as<float>();

    /*[3]--cloud handle ptr init*/
    single_scan_filter_ptr_ = std::make_shared<Module::VoxelFilter>(paramlist_.single_scan_leaf_size);
    local_map_filter_ptr_ = std::make_shared<Module::VoxelFilter>(paramlist_.local_map_leaf_size);
    registration_ptr_ =
        std::make_shared<Module::FastGicpRegistration>(paramlist_.registration_resolution, 0.1, 0.01, 30);
}

/**
 * @brief set initial pos
 * @param[in]
 * @return
 */
bool LidarOdom::InitPose(const Eigen::Matrix4f &lidar_odom_init_pose)
{
    static bool lidar_odom_init_flag_ = false;
    if (lidar_odom_init_flag_ == false)
    {
        lidar_odom_init_pose_ = lidar_odom_init_pose;
        lidar_odom_init_flag_ = true;
    }
    return lidar_odom_init_flag_;
}

/**
 * @brief update lidar odom
 * @param[in out] lidar_odom
 * @param[in] cloud_msg
 * @return
 */
bool LidarOdom::UpdateOdom(Eigen::Matrix4f &lidar_odom, const CloudMsg &cloud_msg)
{

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity(); // pose diff
    static Eigen::Matrix4f last_pose = lidar_odom_init_pose_;
    static Eigen::Matrix4f predict_pose = lidar_odom_init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = lidar_odom_init_pose_;

    /*[1]--create current frame by cloud_msg*/
    FrameMsg current_frame;
    current_frame.cloud_msg.time_stamp = cloud_msg.time_stamp;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_msg.cloud_ptr, *current_frame.cloud_msg.cloud_ptr, indices);

    /*[2]--if first frame*/
    if (local_map_keyframe_msg_queue_.size() == 0)
    {
        lidar_odom = current_frame.pose = lidar_odom_init_pose_;
        UpdateLocalMap(current_frame);
        return true;
    }

    /*[3]--if not first frame*/
    /*a--cloud registration*/
    CloudMsg::CLOUD_PTR filtered_current_scan_ptr(new CloudMsg::CLOUD());
    CloudMsg::CLOUD_PTR filtered_local_map_ptr(new CloudMsg::CLOUD());

    single_scan_filter_ptr_->Filter(current_frame.cloud_msg.cloud_ptr, filtered_current_scan_ptr);
    local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);

    registration_ptr_->SetSourceCloud(filtered_current_scan_ptr);
    registration_ptr_->SetTargetCloud(filtered_local_map_ptr);

    CloudMsg::CLOUD_PTR registered_cloud_ptr(new CloudMsg::CLOUD());
    registration_ptr_->Registration(predict_pose, current_frame.pose, registered_cloud_ptr);
    lidar_odom = current_frame.pose;
    /*b--predict next pose*/
    step_pose = last_pose.inverse() * current_frame.pose; // postmultiplication
    predict_pose = current_frame.pose * step_pose;
    last_pose = current_frame.pose;
    /*c--keyframe extract--*/
    if (fabs(last_key_frame_pose(0, 3) - current_frame.pose(0, 3)) +
            fabs(last_key_frame_pose(1, 3) - current_frame.pose(1, 3)) +
            fabs(last_key_frame_pose(2, 3) - current_frame.pose(2, 3)) >=
        paramlist_.keyframe_distance)
    {
        UpdateLocalMap(current_frame);
        last_key_frame_pose = current_frame.pose;
    }

    return true;
}

/**
 * @brief update locl map
 * @param[in] new_key_frame
 * @return
 */
bool LidarOdom::UpdateLocalMap(const FrameMsg &new_keyframe_msg)
{
    /*[1]--add new keyframe to queue*/
    FrameMsg keyframe_msg = new_keyframe_msg;
    keyframe_msg.cloud_msg.cloud_ptr.reset(new CloudMsg::CLOUD(*new_keyframe_msg.cloud_msg.cloud_ptr)); // deep copy

    local_map_keyframe_msg_queue_.push_back(keyframe_msg);

    /*[2]--maintain the local map queue*/
    while (local_map_keyframe_msg_queue_.size() > static_cast<size_t>(paramlist_.keyframe_num))
    {
        local_map_keyframe_msg_queue_.pop_front();
    }

    /*[3]--stitch localmap cloud*/
    local_map_ptr_.reset(new CloudMsg::CLOUD());
    for (unsigned int i = 0; i < local_map_keyframe_msg_queue_.size(); i++)
    {
        CloudMsg::CLOUD_PTR transformed_cloud_ptr(new CloudMsg::CLOUD());
        pcl::transformPointCloud(*local_map_keyframe_msg_queue_.at(i).cloud_msg.cloud_ptr, *transformed_cloud_ptr,
                                 local_map_keyframe_msg_queue_.at(i).pose);
        *local_map_ptr_ += *transformed_cloud_ptr;
    }
    return true;
}
