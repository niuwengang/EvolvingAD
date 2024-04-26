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
#include "user_msg/frame_msg.hpp"
#include "user_msg/pose_msg.hpp"
// eigen lib
#include <Eigen/Core>
// c++
#include <deque>
// yaml
#include <yaml-cpp/yaml.h>
// module
#include "module/graph_optimizer/g2o/g2o_opter.hpp"
#include "module/graph_optimizer/graph_optimizer_interface.hpp"

class PoseGraph
{
  public:
    PoseGraph(const YAML::Node &config_node);
    bool UpdatePose(const PoseMsg &gnss_odom_msg, const PoseMsg &lidar_odom_msg, PoseMsg &fusion_odom_msg);

  private:
    bool CheckNewKeyFrame(PoseMsg &fusion_odom_msg);
    bool AddVertexandEdge(const PoseMsg &gnss_odom_msg);

  private:
    /*variable*/
    FrameMsg cur_keyframe_msg_;
    std::deque<FrameMsg> keyframe_msg_queue_;
    std::shared_ptr<GraphOptimizerInterface> graph_optimizer_ptr_ = nullptr;
    std::deque<Eigen::Matrix4f> opted_pose_queue_; //  queue opted

    unsigned int new_gnss_cnt_ = 0;
    unsigned int new_loop_cnt_ = 0;
    unsigned int new_keyframe_cnt_ = 0;

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
    } paramlist_;
};

#endif //_POSE_GRAPH_HPP_