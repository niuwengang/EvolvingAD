/**
 * @file    cloud_registration_interface.hpp
 * @brief   gicp
 * @author  niu_wengang@163.com
 * @date    2024-03-30
 * @version 0.1.1
 */

#include "fast_gicp_registration.hpp"
namespace Module
{

/**
 * @brief Ndt配准设置
 * @param[in]
 * @param[in]
 * @return
 */
FastGicpRegistration::FastGicpRegistration(const float resolution, const float step_size, const float trans_eps,
                                           const int max_iter)
{
    gicp_.setResolution(resolution);
    gicp_.setNumThreads(omp_get_max_threads());
}

/**
 * @brief Ndt配准 源点云
 * @param[in]
 * @param[in]
 * @return
 */
void FastGicpRegistration::SetSourceCloud(const CloudMsg::CLOUD_PTR &source_cloud_ptr)
{
    gicp_.setInputSource(source_cloud_ptr);
}

/**
 * @brief Ndt配准 目标点云
 * @param[in]
 * @param[in]
 * @return
 */
void FastGicpRegistration::SetTargetCloud(const CloudMsg::CLOUD_PTR &target_cloud_ptr)
{
    gicp_.setInputTarget(target_cloud_ptr);
}

/**
 * @brief Ndt配准
 * @param[in]
 * @param[in]
 * @return
 */
bool FastGicpRegistration::Registration(const Eigen::Matrix4f &predict_pose, Eigen::Matrix4f &result_pose,
                                        CloudMsg::CLOUD_PTR &result_cloud_ptr)
{
    gicp_.align(*result_cloud_ptr, predict_pose);
    result_pose = gicp_.getFinalTransformation();
    return true;
}

} // namespace Module