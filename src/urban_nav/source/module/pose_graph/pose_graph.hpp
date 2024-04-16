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
// module
#include "module/graph_optimizer/g2o/g2o_opter.hpp"
#include "module/graph_optimizer/graph_optimizer_interface.hpp"

class PoseGraph
{
  public:
    PoseGraph();
    bool UpdatePose(const PoseMsg &gnss_msg, const PoseMsg &lidar_odom_msg, PoseMsg &fusion_odom_msg);

  private:
    bool CheckNewKeyFrame();
    bool AddVertexandEdge(const PoseMsg &gnss_odom_msg);

  private:
    PoseMsg fusion_odom_msg_; // fusion odom
    Eigen::Matrix4f lidar2gnss_transform_ = Eigen::Matrix4f::Identity();

    std::deque<FrameMsg> keyframe_msg_queue_;
    FrameMsg cur_keyframe_msg_;

    std::shared_ptr<GraphOptimizerInterface> graph_optimizer_ptr_ = nullptr;

    std::deque<Eigen::Matrix4f> opted_pose_queue_; // 优化后的队列
};

#endif //_POSE_GRAPH_HPP_