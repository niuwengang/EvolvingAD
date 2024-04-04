/**
 * @file    cloud_registration_interface.hpp
 * @brief   点云滤波虚接口cloud_registration_interface
 * @author  niu_wengang@163.com
 * @date    2024-03-30
 * @version 1.0
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
    vgicp.setResolution(1.0);
    vgicp.setNumThreads(omp_get_max_threads());
}

/**
 * @brief Ndt配准 源点云
 * @param[in]
 * @param[in]
 * @return
 */
void FastGicpRegistration::SetSourceCloud(const CloudMsg::CLOUD_PTR &source_cloud_ptr)
{
    vgicp.setInputSource(source_cloud_ptr);
}

/**
 * @brief Ndt配准 目标点云
 * @param[in]
 * @param[in]
 * @return
 */
void FastGicpRegistration::SetTargetCloud(const CloudMsg::CLOUD_PTR &target_cloud_ptr)
{
    vgicp.setInputTarget(target_cloud_ptr);
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
    vgicp.align(*result_cloud_ptr, predict_pose);
    result_pose = vgicp.getFinalTransformation();
    return true;
}

} // namespace Module