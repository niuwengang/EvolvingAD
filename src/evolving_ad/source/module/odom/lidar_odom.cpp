#include "lidar_odom.hpp"

namespace evolving_ad_ns
{

LidarOdom::LidarOdom(const YAML::Node &config_node)
{
    /*[1]--load paramlist*/
    paramlist_.keyframe_distance = config_node["keyframe_distance"].as<float>();
    paramlist_.keyframe_num = config_node["keyframe_num"].as<int>();
    paramlist_.single_scan_leaf_size = config_node["single_scan_leaf_size"].as<float>();
    paramlist_.local_map_leaf_size = config_node["local_map_leaf_size"].as<float>();
    paramlist_.registration_resolution = config_node["registration_resolution"].as<float>();

    single_scan_filter_ptr_ = std::make_shared<VoxelFilter>(paramlist_.single_scan_leaf_size);
    local_map_filter_ptr_ = std::make_shared<VoxelFilter>(paramlist_.local_map_leaf_size);
    registration_ptr_ = std::make_shared<FastGicpRegistration>(paramlist_.registration_resolution, 0.1, 0.01, 30);

    local_map_ptr_.reset(new CloudMsg::CLOUD());
}

bool LidarOdom::InitPose(const Eigen::Matrix4f &init_pose)
{

    if (init_flag_ == false)
    {
        init_pose_ = init_pose;
        init_flag_ = true;
    }
    return init_flag_;
}

void LidarOdom::ComputePose(const CloudMsg &cloud_msg, Eigen::Matrix4f &new_pose)
{
    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity(); // pose diff
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = init_pose_;

    Frame normal_frame;
    normal_frame.time_stamp = cloud_msg.time_stamp;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_msg.cloud_ptr, *normal_frame.cloud_msg.cloud_ptr, indices);

    /*[1]--first frame*/
    if (local_keyframe_queue_.size() == 0)
    {
        new_pose = normal_frame.pose = init_pose_;
        UpdateLocalMap(normal_frame);
        return;
    }

    /*[2]--not first frame*/
    /*a--cloud registration*/
    CloudMsg::CLOUD_PTR filtered_current_scan_ptr(new CloudMsg::CLOUD());
    CloudMsg::CLOUD_PTR filtered_local_map_ptr(new CloudMsg::CLOUD());

    single_scan_filter_ptr_->Filter(normal_frame.cloud_msg.cloud_ptr, filtered_current_scan_ptr);
    local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);

    registration_ptr_->SetSourceCloud(filtered_current_scan_ptr);
    registration_ptr_->SetTargetCloud(filtered_local_map_ptr);

    CloudMsg::CLOUD_PTR registered_cloud_ptr(new CloudMsg::CLOUD());
    registration_ptr_->Registration(predict_pose, normal_frame.pose, registered_cloud_ptr);
    new_pose = normal_frame.pose;

    /*b--predict next pose*/
    step_pose = last_pose.inverse() * normal_frame.pose; // postmultiplication
    predict_pose = normal_frame.pose * step_pose;
    last_pose = normal_frame.pose;

    if (fabs(last_key_frame_pose(0, 3) - normal_frame.pose(0, 3)) +
            fabs(last_key_frame_pose(1, 3) - normal_frame.pose(1, 3)) +
            fabs(last_key_frame_pose(2, 3) - normal_frame.pose(2, 3)) >=
        paramlist_.keyframe_distance)
    {
        UpdateLocalMap(normal_frame);
        last_key_frame_pose = normal_frame.pose;
    }
    return;
}

void LidarOdom::UpdateLocalMap(const Frame &normal_frame)
{
    /*[1]--add new keyframe to queue*/
    Frame keyframe = normal_frame;
    local_keyframe_queue_.push_back(keyframe);

    /*[2]--maintain the local map queue*/
    while (local_keyframe_queue_.size() > static_cast<size_t>(paramlist_.keyframe_num))
    {
        local_keyframe_queue_.pop_front();
    }

    /*[3]--stitch localmap cloud*/ //! accel with omp
    local_map_ptr_.reset(new CloudMsg::CLOUD());
    for (unsigned int i = 0; i < local_keyframe_queue_.size(); i++)
    {
        CloudMsg::CLOUD_PTR transformed_cloud_ptr(new CloudMsg::CLOUD());
        pcl::transformPointCloud(*local_keyframe_queue_.at(i).cloud_msg.cloud_ptr, *transformed_cloud_ptr,
                                 local_keyframe_queue_.at(i).pose);
        *local_map_ptr_ += *transformed_cloud_ptr;
    }
}

} // namespace evolving_ad_ns