#ifndef _GRAPH_OPTIMIZER_INTERFACE_HPP_
#define _GRAPH_OPTIMIZER_INTERFACE_HPP_

#include "user_msg/pose_msg.hpp"
// c++ lib
#include <deque>

class GraphOptimizerInterface
{
  public:
    virtual ~GraphOptimizerInterface()
    {
    }
    /*exec opt*/
    virtual bool Opimtize() = 0;
    /*add se3 vertex*/
    virtual void AddSe3Vertex(const Eigen::Isometry3d &pose, const bool is_fixed) = 0;

    /*add prior xyz edge*/
    virtual void AddPriorXYZEdge(const unsigned int vertex_index, const Eigen::Vector3d &xyz,
                                 const std::vector<double> &noise_vec) = 0;

    virtual void AddPriorQuatEdge(const unsigned int vertex_index, const Eigen::Quaterniond &quat,
                                  const std::vector<double> &noise_vec) = 0;

    virtual void AddInteriorSe3Edge(const unsigned int vertex_index_0, const unsigned int vertex_index_1,
                                    const Eigen::Isometry3d &relative_pose, const std::vector<double> &noise_vec) = 0;

    /*acquire data*/
    virtual void GetOptPoseQueue(std::deque<Eigen::Matrix4f> &opted_pose_queue) = 0;
    virtual unsigned int GetOptNodeNum() = 0;

  protected:
    unsigned int max_iterations_num_ = 50;
};

#endif