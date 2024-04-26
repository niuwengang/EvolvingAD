/**
 * @file    pose_graph.cpp
 * @brief   图优化构建
 * @author  niu_wengang@163.com
 * @date    2024-04-08
 * @version 0.1.2
 */

#include "pose_graph.hpp"
#include "tools/file_manager/file_manager.hpp"

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

    paramlist_.result_folfer = config_node["result_folfer"].as<std::string>();

    /*[2]--create folder*/
    if (paramlist_.result_folfer == "")
    {
        paramlist_.result_folfer = ros::package::getPath("evolving_ad") + "/result";
    }
    paramlist_.result_subfolfer_keyframe = paramlist_.result_folfer + "/keyframe";
    paramlist_.result_subfolfer_trajectory = paramlist_.result_folfer + "/trajectory";

    Tools::FileManager::CreateFolder(paramlist_.result_folfer);
    Tools::FileManager::CreateFolder(paramlist_.result_subfolfer_keyframe);
    Tools::FileManager::CreateFolder(paramlist_.result_subfolfer_trajectory);

    Tools::FileManager::CreateTxtFile(gnss_odom_ofs_, paramlist_.result_subfolfer_trajectory + "/gnss_traj.txt");
    Tools::FileManager::CreateTxtFile(lidar_odom_ofs_, paramlist_.result_subfolfer_trajectory + "/lidar_traj.txt");
    Tools::FileManager::CreateTxtFile(opt_odom_ofs_, paramlist_.result_subfolfer_trajectory + "/opt_traj.txt");
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
bool PoseGraph::UpdatePose(const CloudMsg &cloud_msg, const PoseMsg &lidar_odom_msg, const PoseMsg &gnss_odom_msg)
{

    /*[1]--reset */
    has_new_keyframe_flag_ = false;
    has_new_optimize_flag_ = false;

    if (CheckNewKeyFrame(cloud_msg, lidar_odom_msg, gnss_odom_msg) == true)
    {

        AddVertexandEdge(gnss_odom_msg);

        SaveTrajectory(lidar_odom_msg.pose, lidar_odom_ofs_);
        SaveTrajectory(gnss_odom_msg.pose, gnss_odom_ofs_);

        if (new_keyframe_cnt_ >= paramlist_.new_keyframe_cnt_max)
        {
            new_keyframe_cnt_ = 0;
            if (graph_optimizer_ptr_->Opimtize() == true)
            {
                graph_optimizer_ptr_->GetOptPoseQueue(opted_pose_msg_queue_);
            }
        }
        return true;
    }
}

void PoseGraph::FinalOptimize()
{
    if (graph_optimizer_ptr_->Opimtize() == true)
    {
        graph_optimizer_ptr_->GetOptPoseQueue(opted_pose_msg_queue_);

        for (const auto &pose_msg : opted_pose_msg_queue_)
        {
            SaveTrajectory(pose_msg.pose, opt_odom_ofs_);
        }
    } // todo bool
}

/**
 * @brief check if keyframe
 * @param[in]
 * @return
 */
bool PoseGraph::CheckNewKeyFrame(const CloudMsg &cloud_msg, const PoseMsg &lidar_odom_msg, const PoseMsg &gnss_odom_msg)
{

    static Eigen::Matrix4f last_keyframe_pose = lidar_odom_msg.pose;

    /*[1]--check if keyframe*/
    if ((fabs(lidar_odom_msg.pose(0, 3) - last_keyframe_pose(0, 3)) +
             fabs(lidar_odom_msg.pose(1, 3) - last_keyframe_pose(1, 3)) +
             fabs(lidar_odom_msg.pose(2, 3) - last_keyframe_pose(2, 3)) >=
         paramlist_.keyframe_distance) or
        keyframe_msg_queue_.empty())
    {
        has_new_keyframe_flag_ = true; // set flag
        last_keyframe_pose = lidar_odom_msg.pose;
    }

    /*[2]--if has new keyframe */
    if (has_new_keyframe_flag_ == true)
    {
        std::string pcd_file_path =
            paramlist_.result_subfolfer_keyframe + "/keyframe_" + std::to_string(keyframe_msg_queue_.size()) + ".pcd";
        pcl::io::savePCDFileBinary(pcd_file_path, *cloud_msg.cloud_ptr);

        FrameMsg keyframe_msg;
        keyframe_msg.time_stamp = lidar_odom_msg.time_stamp;
        keyframe_msg.pose = lidar_odom_msg.pose;
        keyframe_msg.index = static_cast<unsigned int>(keyframe_msg_queue_.size()); // index=size-1
        keyframe_msg_queue_.push_back(keyframe_msg);

        cur_keyframe_msg_ = keyframe_msg; // only shallow copy

        cur_keygnss_msg_.index = keyframe_msg.index;
        cur_keygnss_msg_.time_stamp = gnss_odom_msg.time_stamp;
        cur_keygnss_msg_.pose = gnss_odom_msg.pose;
    }

    return has_new_keyframe_flag_;
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

    return true;
}

void PoseGraph::GetOptedPoseQueue(std::deque<PoseMsg> &opted_pose_msg_queue)
{
    opted_pose_msg_queue.clear();
    opted_pose_msg_queue = this->opted_pose_msg_queue_;
}

/**
 * @brief trajsave
 * @param[in]
 * @return
 */
void PoseGraph::SaveTrajectory(const Eigen::Matrix4f &target_odom, std::ofstream &ofs)
{

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            ofs << target_odom(i, j);
            if (i == 2 && j == 3)
            {
                ofs << std::endl;
            }
            else
            {
                ofs << " ";
            }
        }
    }
}