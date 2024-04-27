/**
 * @file    pose_graph.hpp
 * @brief   build pose graph
 * @author  niu_wengang@163.com
 * @date    2024-04-08
 * @version 0.1.2
 */

#ifndef _POSE_GRAPH_HPP_
#define _POSE_GRAPH_HPP_

// user msg lib
#include "user_msg/cloud_msg.hpp"
#include "user_msg/frame_msg.hpp"
#include "user_msg/pose_msg.hpp"
// eigen lib
#include <Eigen/Core>
// c++
#include <deque>
// yaml
#include <yaml-cpp/yaml.h>
// module
#include "module/cloud_filter/cloud_filter_interface.hpp"
#include "module/cloud_filter/voxel_filter.hpp"
#include "module/graph_optimizer/g2o/g2o_opter.hpp"
#include "module/graph_optimizer/graph_optimizer_interface.hpp"
// ros
#include <ros/package.h>
// pcl
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

class PoseGraph
{
  public:
    PoseGraph(const YAML::Node &config_node);
    bool UpdatePose(const CloudMsg &cloud_msg, const PoseMsg &lidar_odom_msg, const PoseMsg &gnss_odom_msg);
    void FinalOptimize();
    void GetOptedPoseQueue(std::deque<PoseMsg> &opted_pose_msg_queue);
    void GetGnss2Lidar(const Eigen::Matrix4f &transform);

  private:
    bool CheckNewKeyFrame(const CloudMsg &cloud_msg, const PoseMsg &lidar_odom_msg, const PoseMsg &gnss_odom_msg);
    bool AddVertexandEdge(const PoseMsg &gnss_odom_msg);
    void SaveTrajectory(const PoseMsg &pose_msg, std::ofstream &ofs, const double time_stamp_increment);
    void SaveAllMap(const std::deque<PoseMsg> &pose_msg_queue, CloudMsg::CLOUD_PTR &all_map_ptr);

  private:
    /*variable*/
    FrameMsg cur_keyframe_msg_;
    FrameMsg cur_keygnss_msg_;
    std::deque<FrameMsg> keyframe_msg_queue_;
    std::deque<PoseMsg> opted_pose_msg_queue_;

    bool has_new_keyframe_flag_ = false;
    bool has_new_optimize_flag_ = false;

    unsigned int new_gnss_cnt_ = 0;
    unsigned int new_loop_cnt_ = 0;
    unsigned int new_keyframe_cnt_ = 0;

    std::ofstream gnss_odom_ofs_;
    std::ofstream lidar_odom_ofs_;
    std::ofstream opt_odom_ofs_;

    /*algorithm module*/
    std::shared_ptr<GraphOptimizerInterface> graph_optimizer_ptr_ = nullptr;

    Eigen::Matrix4f gnss_to_lidar_;
    struct ParamLists
    {
      public:
        /*noise matrix*/
        std::vector<double> gnss_odom_noise;
        std::vector<double> lidar_odom_noise;
        std::vector<double> imu_odom_noise;
        /*opt cnt*/
        unsigned int new_gnss_cnt_max = 0;
        unsigned int new_loop_cnt_max = 0;
        unsigned int new_keyframe_cnt_max = 0;

        double keyframe_distance = 2.0;

        std::string result_folfer = "";
        std::string result_subfolfer_keyframe = "";
        std::string result_subfolfer_trajectory = "";

        Eigen::Matrix4f lidar_to_body = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f imu_to_body = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f cam_to_lidar = Eigen::Matrix4f::Identity();

    } paramlist_;
};

#endif //_POSE_GRAPH_HPP_