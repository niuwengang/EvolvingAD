#ifndef _GRAPH_OPTIMIZER_INTERFACE_HPP_
#define _GRAPH_OPTIMIZER_INTERFACE_HPP_

// c++ lib
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>

namespace evolving_ad_ns
{

class GraphOptimizerInterface
{
  public:
    virtual ~GraphOptimizerInterface()
    {
    }
    /*exec opt*/
    virtual bool Opimtize() = 0;
    /*vertex*/
    virtual void AddSe3Vertex(const Eigen::Isometry3d &pose, const bool is_fixed) = 0;

    /*edge*/
    virtual void AddPriorXYZEdge(const unsigned int vertex_index, const Eigen::Vector3d &xyz,
                                 const std::array<double, 3> &noise_array) = 0;

    virtual void AddPriorQuatEdge(const unsigned int vertex_index, const Eigen::Quaterniond &quat,
                                  const std::array<double, 4> &noise_array) = 0;

    virtual void AddInteriorSe3Edge(const unsigned int vertex_index_0, const unsigned int vertex_index_1,
                                    const Eigen::Isometry3d &relative_pose,
                                    const std::array<double, 6> &noise_array) = 0; // xyz rpy

    /*acquire data*/
    virtual void GetOptPoseQueue(std::deque<Eigen::Isometry3d> &opted_pose_queue) = 0;
    virtual unsigned int GetOptNodeNum() = 0;

  protected:
    unsigned int max_iterations_num_ = 50;
};
} // namespace evolving_ad_ns

#endif