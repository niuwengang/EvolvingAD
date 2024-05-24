#include "lidar_odom.hpp"

namespace evolving_ad_ns
{

LidarOdom::LidarOdom(const YAML::Node &config_node)
{
    /*[1]--load paramlist*/
    paramlist_.keyframe_distance = config_node["keyframe_distance"].as<float>();
    paramlist_.keyframe_num = config_node["keyframe_num"].as<int>();
    paramlist_.filter_leaf_size_small = config_node["filter_leaf_size_small"].as<float>();
    paramlist_.filter_leaf_size_media = config_node["filter_leaf_size_media"].as<float>();
    paramlist_.filter_leaf_size_large = config_node["filter_leaf_size_large"].as<float>();

    paramlist_.registration_resolution = config_node["registration_resolution"].as<float>();

    filter_small_size_ptr_ = std::make_shared<VoxelFilter>(paramlist_.filter_leaf_size_small);
    filter_media_size_ptr_ = std::make_shared<VoxelFilter>(paramlist_.filter_leaf_size_media);
    filter_large_size_ptr_ = std::make_shared<VoxelFilter>(paramlist_.filter_leaf_size_large);

    registration_ptr_ = std::make_shared<FastGicpRegistration>(paramlist_.registration_resolution, 0.1, 0.01, 30);

    local_map_ptr_.reset(new CloudMsg::CLOUD());
}

/**
 * @brief set inital pos
 * @param[in] init_pose
 * @return if has been inited
 * @note forbid to use static value here
 */
bool LidarOdom::InitPose(const Eigen::Matrix4f &init_pose)
{
    if (init_flag_ == false)
    {
        last_keyframe_pose_ = init_pose_ = init_pose;
        init_flag_ = true;
    }
    return init_flag_;
}

/**
 * @brief corse align
 * @param[in] cloud_msg
 * @param[in] corse_pose
 * @note scan to scan for corse align
 */
void LidarOdom::ComputeCorsePose(const CloudMsg &cloud_msg, const Eigen::Matrix4f &imu_pose,
                                 Eigen::Matrix4f &corse_pose) // add imu
{

    if (first_flag_ == false) // not use size to judge
    {
        CloudMsg::CLOUD_PTR cur_scan_ptr(new CloudMsg::CLOUD());
        filter_small_size_ptr_->Filter(cloud_msg.cloud_ptr, cur_scan_ptr); // lidar coordinate

        registration_ptr_->SetSourceCloud(cloud_msg_pre_.cloud_ptr);
        registration_ptr_->SetTargetCloud(cur_scan_ptr);

        Eigen::Matrix4f predict_pose = Eigen::Matrix4f::Identity();
        CloudMsg::CLOUD_PTR registered_cloud_ptr(new CloudMsg::CLOUD());
        registration_ptr_->Registration(imu_pose, corse_pose, registered_cloud_ptr);

        *cloud_msg_pre_.cloud_ptr = *cur_scan_ptr;
    }
    else
    {
        *cloud_msg_pre_.cloud_ptr = *cloud_msg.cloud_ptr;
        corse_pose = init_pose_;
        first_flag_ = false;
    }
}

/**
 * @brief fine align
 * @param[in] cloud_msg
 * @param[in] fine_pose
 * @note scan to map for fine align
 */
void LidarOdom::ComputeFinePose(const CloudMsg &cloud_msg, const Eigen::Matrix4f &corse_pose,
                                Eigen::Matrix4f &fine_pose)
{
    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity(); // pose diff
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;

    if (!local_keyframe_queue_.empty()) // not use size to judge
    {
        /*1--registration*/
        CloudMsg::CLOUD_PTR cur_scan_ptr(new CloudMsg::CLOUD());
        filter_media_size_ptr_->Filter(cloud_msg.cloud_ptr, cur_scan_ptr); // lidar coordinate

        CloudMsg::CLOUD_PTR filtered_local_map_ptr(new CloudMsg::CLOUD());
        filter_large_size_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);

        std::cout << "cur_scan_ptr size:" << cur_scan_ptr->points.size() << std::endl;
        std::cout << "filtered_local_map_ptr size:" << filtered_local_map_ptr->points.size() << std::endl;
        registration_ptr_->SetSourceCloud(cur_scan_ptr);
        registration_ptr_->SetTargetCloud(filtered_local_map_ptr);

        CloudMsg::CLOUD_PTR registered_cloud_ptr(new CloudMsg::CLOUD());

        Eigen::Matrix4f infer_pose = Eigen::Matrix4f::Identity();
        infer_pose.block<3, 3>(0, 0) = (last_pose * corse_pose).block<3, 3>(0, 0); // R
        infer_pose.block<3, 1>(0, 3) = predict_pose.block<3, 1>(0, 3);             // t
        registration_ptr_->Registration(infer_pose, fine_pose, registered_cloud_ptr);

        /*2--predict pose*/
        step_pose = last_pose.inverse() * fine_pose;
        predict_pose = fine_pose * step_pose;
        last_pose = fine_pose;

        /*3--check keyframe*/
        if (fabs(last_keyframe_pose_(0, 3) - fine_pose(0, 3)) + fabs(last_keyframe_pose_(1, 3) - fine_pose(1, 3)) +
                fabs(last_keyframe_pose_(2, 3) - fine_pose(2, 3)) >=
            paramlist_.keyframe_distance)
        {
            Frame frame;
            frame.cloud_msg = cloud_msg;
            frame.pose = fine_pose;
            local_keyframe_queue_.push_back(frame);

            while (local_keyframe_queue_.size() > static_cast<size_t>(paramlist_.keyframe_num))
            {
                local_keyframe_queue_.pop_front();
            }

            local_map_ptr_.reset(new CloudMsg::CLOUD());
            for (unsigned int i = 0; i < local_keyframe_queue_.size(); i++)
            {
                CloudMsg::CLOUD_PTR transformed_cloud_ptr(new CloudMsg::CLOUD());
                pcl::transformPointCloud(*local_keyframe_queue_.at(i).cloud_msg.cloud_ptr, *transformed_cloud_ptr,
                                         local_keyframe_queue_.at(i).pose);
                *local_map_ptr_ += *transformed_cloud_ptr;
            }
            last_keyframe_pose_ = fine_pose;
        }
    }
    else
    {
        Frame frame;
        frame.cloud_msg = cloud_msg;
        frame.pose = init_pose_;
        local_keyframe_queue_.push_back(frame);

        local_map_ptr_.reset(new CloudMsg::CLOUD());
        CloudMsg::CLOUD_PTR transformed_cloud_ptr(new CloudMsg::CLOUD());
        pcl::transformPointCloud(*local_keyframe_queue_.front().cloud_msg.cloud_ptr, *transformed_cloud_ptr,
                                 local_keyframe_queue_.front().pose);
        *local_map_ptr_ += *transformed_cloud_ptr;
    }
}

// void LidarOdom::ComputePose(const CloudMsg &cloud_msg, Eigen::Matrix4f &new_pose)
// {

//     // static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity(); // pose diff
//     // static Eigen::Matrix4f last_pose = init_pose_;
//     // static Eigen::Matrix4f predict_pose = init_pose_;
//     // static Eigen::Matrix4f last_key_frame_pose = init_pose_;

//     // Frame normal_frame;
//     // normal_frame.time_stamp = cloud_msg.time_stamp;
//     // std::vector<int> indices;
//     // pcl::removeNaNFromPointCloud(*cloud_msg.cloud_ptr, *normal_frame.cloud_msg.cloud_ptr, indices);

//     // /*[1]--first frame*/
//     // if (local_keyframe_queue_.size() == 0)
//     // {
//     //     new_pose = normal_frame.pose = init_pose_;
//     //     UpdateLocalMap(normal_frame);
//     //     return;
//     // }

//     // /*[2]--not first frame*/
//     // /*a--cloud registration*/
//     // CloudMsg::CLOUD_PTR filtered_current_scan_ptr(new CloudMsg::CLOUD());
//     // CloudMsg::CLOUD_PTR filtered_local_map_ptr(new CloudMsg::CLOUD());

//     // single_scan_filter_ptr_->Filter(normal_frame.cloud_msg.cloud_ptr, filtered_current_scan_ptr);
//     // local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);

//     // registration_ptr_->SetSourceCloud(filtered_current_scan_ptr);
//     // registration_ptr_->SetTargetCloud(filtered_local_map_ptr);

//     // CloudMsg::CLOUD_PTR registered_cloud_ptr(new CloudMsg::CLOUD());
//     // registration_ptr_->Registration(predict_pose, normal_frame.pose, registered_cloud_ptr);
//     // new_pose = normal_frame.pose;

//     // /*b--predict next pose*/
//     // step_pose = last_pose.inverse() * normal_frame.pose; // postmultiplication
//     // predict_pose = normal_frame.pose * step_pose;
//     // last_pose = normal_frame.pose;

//     // if (fabs(last_key_frame_pose(0, 3) - normal_frame.pose(0, 3)) +
//     //         fabs(last_key_frame_pose(1, 3) - normal_frame.pose(1, 3)) +
//     //         fabs(last_key_frame_pose(2, 3) - normal_frame.pose(2, 3)) >=
//     //     paramlist_.keyframe_distance)
//     // {
//     //     UpdateLocalMap(normal_frame);
//     //     last_key_frame_pose = normal_frame.pose;
//     // }
//     // return;
// }

// void LidarOdom::UpdateLocalMap(const Frame &normal_frame)
// {
//     // /*[1]--add new keyframe to queue*/
//     // Frame keyframe = normal_frame;
//     // local_keyframe_queue_.push_back(keyframe);

//     // /*[2]--maintain the local map queue*/
//     // while (local_keyframe_queue_.size() > static_cast<size_t>(paramlist_.keyframe_num))
//     // {
//     //     local_keyframe_queue_.pop_front();
//     // }

//     // /*[3]--stitch localmap cloud*/ //! accel with omp
//     // local_map_ptr_.reset(new CloudMsg::CLOUD());
//     // for (unsigned int i = 0; i < local_keyframe_queue_.size(); i++)
//     // {
//     //     CloudMsg::CLOUD_PTR transformed_cloud_ptr(new CloudMsg::CLOUD());
//     //     pcl::transformPointCloud(*local_keyframe_queue_.at(i).cloud_msg.cloud_ptr, *transformed_cloud_ptr,
//     //                              local_keyframe_queue_.at(i).pose);
//     //     *local_map_ptr_ += *transformed_cloud_ptr;
//     // }
// }

} // namespace evolving_ad_ns