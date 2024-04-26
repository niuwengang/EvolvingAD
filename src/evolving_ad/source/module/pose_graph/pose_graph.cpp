/**
 * @file    pose_graph.cpp
 * @brief   图优化构建
 * @author  niu_wengang@163.com
 * @date    2024-04-08
 * @version 0.1.2
 */

#include "pose_graph.hpp"

/**
 * @brief pose graph build
 * @param[in]
 * @return
 */
PoseGraph::PoseGraph(const YAML::Node &config_node)
{
    /*[1]--load params*/
    paramlist_.new_gnss_cnt_max = config_node["new_gnss_cnt_max"].as<int>();
    paramlist_.new_loop_cnt_max = config_node["new_loop_cnt_max"].as<int>();
    paramlist_.new_keyframe_cnt_max = config_node["new_key_frame_cnt_max"].as<int>();

    paramlist_.gnss_odom_noise = config_node["gnss_odom_noise"].as<std::vector<double>>();
    paramlist_.lidar_odom_noise = config_node["lidar_odom_noise"].as<std::vector<double>>();
    paramlist_.imu_odom_noise = config_node["imu_odom_noise"].as<std::vector<double>>();

    paramlist_.keyframe_distance = config_node["keyframe_distance"].as<double>();

    paramlist_.result_save_folfer = config_node["result_save_folfer"].as<std::string>();

    /*[2]--create folder*/
    if (paramlist_.result_save_folfer == "")
    {
        paramlist_.result_save_folfer = ros::package::getPath("evolving_ad") + "/result";
    }

    /*[3]-graph optimizer setting*/
    graph_optimizer_ptr_ = std::make_shared<G2oOpter>("lm_var");
}

/**
 * @brief pose update
 * @param[in] gnss_msg
 * @param[in] lidar_odom_msg
 * @param[in] fusion_odom_msg
 * @return
 */
bool PoseGraph::UpdatePose(const PoseMsg &gnss_odom_msg, const PoseMsg &lidar_odom_msg, PoseMsg &fusion_odom_msg)
{
    /*1--build fusion odom use lidar odom(has been transformed to gnss coor)*/
    fusion_odom_msg.pose = lidar_odom_msg.pose; // lidar pose is basic
    fusion_odom_msg.time_stamp = lidar_odom_msg.time_stamp;

    if (CheckNewKeyFrame(fusion_odom_msg) == true)
    {

        AddVertexandEdge(gnss_odom_msg);
        if (graph_optimizer_ptr_->Opimtize() == true)
        {
            graph_optimizer_ptr_->GetOptPoseQueue(opted_pose_queue_);
            fusion_odom_msg.pose = opted_pose_queue_.back(); // just temp
        }
        // if (new_keyframe_cnt_ >= paramlist_.new_keyframe_cnt_max)
        // {
        //     new_keyframe_cnt_ = 0;

        // }
    }
    return true;
}

/**
 * @brief check if keyframe
 * @param[in]
 * @return
 */
bool PoseGraph::CheckNewKeyFrame(PoseMsg &fusion_odom_msg)
{

    static Eigen::Matrix4f last_keyframe_pose = fusion_odom_msg.pose;

    bool has_new_keyframe_flag = false;
    /*[1]--check if keyframe*/
    if (keyframe_msg_queue_.size() == 0)
    {
        has_new_keyframe_flag = true; // set flag
        last_keyframe_pose = fusion_odom_msg.pose;
    }
    if (fabs(fusion_odom_msg.pose(0, 3) - last_keyframe_pose(0, 3)) +
            fabs(fusion_odom_msg.pose(1, 3) - last_keyframe_pose(1, 3)) +
            fabs(fusion_odom_msg.pose(2, 3) - last_keyframe_pose(2, 3)) >=
        paramlist_.keyframe_distance)
    {
        last_keyframe_pose = fusion_odom_msg.pose;
        has_new_keyframe_flag = true; // set flag
    }

    /*[2]--if has new keyframe */
    if (has_new_keyframe_flag == true)
    {
        FrameMsg keyframe_msg;
        keyframe_msg.time_stamp = fusion_odom_msg.time_stamp;
        keyframe_msg.pose = fusion_odom_msg.pose;
        keyframe_msg.index = static_cast<unsigned int>(keyframe_msg_queue_.size()); // index=size-1
        keyframe_msg_queue_.push_back(keyframe_msg);
        cur_keyframe_msg_ = keyframe_msg; // only shallow copy
    }

    return has_new_keyframe_flag;
}

bool PoseGraph::AddVertexandEdge(const PoseMsg &gnss_odom_msg)
{
    static Eigen::Matrix4f last_keyframe_pose = cur_keyframe_msg_.pose; // for diff pose

    /*[1]--add vertex*/
    Eigen::Isometry3d isometry;
    isometry.matrix() = cur_keyframe_msg_.pose.cast<double>();
    if (graph_optimizer_ptr_->GetOptNodeNum() == 0) // first must be fixed
    {
        graph_optimizer_ptr_->AddSe3Vertex(isometry, true);
    }
    else
    {
        graph_optimizer_ptr_->AddSe3Vertex(isometry, false);
    }
    new_keyframe_cnt_++;

    /*[2]--add interior constraints*/
    int node_num = graph_optimizer_ptr_->GetOptNodeNum();
    if (node_num >= 2) // at least two nodes
    {
        Eigen::Matrix4f relative_pose = last_keyframe_pose.inverse() * cur_keyframe_msg_.pose;
        isometry.matrix() = relative_pose.cast<double>();
        graph_optimizer_ptr_->AddInteriorSe3Edge(node_num - 2, node_num - 1, isometry, paramlist_.lidar_odom_noise);
    }
    last_keyframe_pose = cur_keyframe_msg_.pose;

    /*[3]--add priorXYZ constraints*/
    Eigen::Vector3d xyz(static_cast<double>(gnss_odom_msg.pose(0, 3)), static_cast<double>(gnss_odom_msg.pose(1, 3)),
                        static_cast<double>(gnss_odom_msg.pose(2, 3)));
    graph_optimizer_ptr_->AddPriorXYZEdge(node_num - 1, xyz, paramlist_.gnss_odom_noise);
    // new_gnss_cnt_++;

    return true;
}
