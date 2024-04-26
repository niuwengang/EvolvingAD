/**
 * @file    g2o_opter.hpp
 * @brief   opt use g2o
 * @author  niu_wengang@163.com
 * @date    2024-04-08
 * @version 0.1.2
 */

#ifndef _G2O_OPTER_HPP_
#define _G2O_OPTER_HPP_

#include "../graph_optimizer_interface.hpp"
// g2o
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/stuff/macros.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
// c++
#include <memory>

// spflog
#include <spdlog/spdlog.h>

#include "edge/edge_se3_priorquat.hpp"
#include "edge/edge_se3_priorxyz.hpp"

G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
G2O_USE_OPTIMIZATION_LIBRARY(csparse)
G2O_USE_TYPE_GROUP(slam3d);

class G2oOpter : public GraphOptimizerInterface
{
  public:
    G2oOpter(const std::string &solver_type = "lm_var");
    /*exec opt*/
    bool Opimtize();
    /*add vertex*/
    void AddSe3Vertex(const Eigen::Isometry3d &pose, const bool is_fixed) override;

    /*add edge*/
    void AddPriorXYZEdge(const unsigned int vertex_index, const Eigen::Vector3d &xyz,
                         const std::vector<double> &noise_vec) override;

    void AddPriorQuatEdge(const unsigned int vertex_index, const Eigen::Quaterniond &quat,
                          const std::vector<double> &noise_vec) override;

    void AddInteriorSe3Edge(const unsigned int vertex_index_0, const unsigned int vertex_index_1,
                            const Eigen::Isometry3d &relative_pose, const std::vector<double> &noise_vec) override;

    /*get result*/
    void GetOptPoseQueue(std::deque<PoseMsg> &opted_pose_msg_queue) override;
    unsigned int GetOptNodeNum() override;

  private:
    g2o::RobustKernelFactory *robust_kernel_factory_;
    std::unique_ptr<g2o::SparseOptimizer> g2o_opter_ptr_ = nullptr;
};

#endif