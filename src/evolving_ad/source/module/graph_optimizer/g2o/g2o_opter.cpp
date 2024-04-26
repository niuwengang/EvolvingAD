#include "g2o_opter.hpp"

/**
 * @brief g2o init
 * @param[in] solver_type opt method
 * @return
 */
G2oOpter::G2oOpter(const std::string &solver_type)
{
    g2o_opter_ptr_.reset(new g2o::SparseOptimizer());

    g2o::OptimizationAlgorithmFactory *solver_factory = g2o::OptimizationAlgorithmFactory::instance();
    g2o::OptimizationAlgorithmProperty solver_property;
    g2o::OptimizationAlgorithm *solver = solver_factory->construct(solver_type, solver_property);

    g2o_opter_ptr_->setAlgorithm(solver);

    if (!g2o_opter_ptr_->solver())
    {
        spdlog::error("g2o_opter$ start faile:{}", solver_type);
        exit(EXIT_FAILURE);
    }
    spdlog::info("g2o_opter$ start success:{}", solver_type);
    robust_kernel_factory_ = g2o::RobustKernelFactory::instance();
}

/**
 * @brief exec opt
 * @param[in]
 * @return
 */
bool G2oOpter::Opimtize()
{

    if (g2o_opter_ptr_->edges().size() < 1)
    {
        return false;
    }

    g2o_opter_ptr_->initializeOptimization();
    g2o_opter_ptr_->computeInitialGuess();
    g2o_opter_ptr_->computeActiveErrors();
    g2o_opter_ptr_->setVerbose(true);

    double chi2 = g2o_opter_ptr_->chi2();
    int iterations = g2o_opter_ptr_->optimize(max_iterations_num_);

    spdlog::info("g2o_opter$ vertices num:{}", g2o_opter_ptr_->vertices().size());
    spdlog::info("g2o_opter$ edge num:{}", g2o_opter_ptr_->edges().size());
    spdlog::info("g2o_opter$ chi before opt:{}:", chi2);
    spdlog::info("g2o_opter$ chi after  opt:{}:", g2o_opter_ptr_->chi2());
    spdlog::info("g2o_opter$ opt cnts: {}", iterations);
    spdlog::info("---------------------------------------");

    return true;
}

/**
 * @brief add vertex
 * @param[in]
 * @return
 */
void G2oOpter::AddSe3Vertex(const Eigen::Isometry3d &pose, const bool is_fixed)
{
    g2o::VertexSE3 *vertex(new g2o::VertexSE3());

    vertex->setId(g2o_opter_ptr_->vertices().size());
    vertex->setEstimate(pose);
    if (is_fixed)
    {
        vertex->setFixed(true);
    }

    g2o_opter_ptr_->addVertex(vertex);
}

/**
 * @brief add edge
 * @param[in]
 * @return
 */
void G2oOpter::AddPriorXYZEdge(const unsigned int vertex_index, const Eigen::Vector3d &xyz,
                               const std::vector<double> &noise_vec)
{
    Eigen::Matrix<double, 3, 3> information_matrix = Eigen::Matrix<double, 3, 3>::Identity();
    for (int i = 0; i < 3; i++)
    {
        information_matrix(i, i) /= noise_vec[i];
    }

    g2o::VertexSE3 *vertex = dynamic_cast<g2o::VertexSE3 *>(g2o_opter_ptr_->vertex(vertex_index));
    g2o::EdgeSE3PriorXYZ *edge(new g2o::EdgeSE3PriorXYZ());
    edge->setMeasurement(xyz);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = vertex;
    g2o_opter_ptr_->addEdge(edge);
}

void G2oOpter::AddPriorQuatEdge(const unsigned int vertex_index, const Eigen::Quaterniond &quat,
                                const std::vector<double> &noise_vec)
{
}

/**
 * @brief add edge
 * @param[in]
 * @return
 */
void G2oOpter::AddInteriorSe3Edge(const unsigned int vertex_index_0, const unsigned int vertex_index_1,
                                  const Eigen::Isometry3d &relative_pose, const std::vector<double> &noise_vec)
{
    Eigen::Matrix<double, 6, 6> information_matrix = Eigen::Matrix<double, 6, 6>::Identity();
    for (int i = 0; i < 6; i++)
    {
        information_matrix(i, i) /= noise_vec[i];
    }

    g2o::VertexSE3 *v0 = dynamic_cast<g2o::VertexSE3 *>(g2o_opter_ptr_->vertex(vertex_index_0));
    g2o::VertexSE3 *v1 = dynamic_cast<g2o::VertexSE3 *>(g2o_opter_ptr_->vertex(vertex_index_1));

    g2o::EdgeSE3 *edge(new g2o::EdgeSE3());
    edge->setMeasurement(relative_pose);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v0;
    edge->vertices()[1] = v1;
    g2o_opter_ptr_->addEdge(edge);
}

/**
 * @brief get result
 * @param[in]
 * @return
 */
void G2oOpter::GetOptPoseQueue(std::deque<Eigen::Matrix4f> &opted_pose_queue)
{
    /*1--clear*/
    opted_pose_queue.clear();
    int vertex_num = g2o_opter_ptr_->vertices().size();
    for (int index = 0; index < vertex_num; index++)
    {
        g2o::VertexSE3 *v = dynamic_cast<g2o::VertexSE3 *>(g2o_opter_ptr_->vertex(index));
        Eigen::Isometry3d pose = v->estimate();
        opted_pose_queue.push_back(pose.matrix().cast<float>());
    }
    return;
}

/**
 * @brief get vertex num
 * @param[in]
 * @return
 */
unsigned int G2oOpter::GetOptNodeNum()
{
    return g2o_opter_ptr_->vertices().size();
}