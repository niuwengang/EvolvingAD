/**
 * @file    cloud_registration_interface.hpp
 * @brief   点云滤波虚接口cloud_registration_interface
 * @author  niu_wengang@163.com
 * @date    2024-03-30
 * @version 1.0
 */

#include "ndt_registration.hpp"
namespace Module
{

/**
 * @brief Ndt配准设置
 * @param[in]
 * @param[in]
 * @return
 */
NdtRegistration::NdtRegistration(const float resolution, const float step_size, const float trans_eps,
                                 const int max_iter)
    : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudMsg::POINT, CloudMsg::POINT>())
{
    ndt_ptr_->setResolution(resolution);
    ndt_ptr_->setStepSize(step_size);
    ndt_ptr_->setTransformationEpsilon(trans_eps);
    ndt_ptr_->setMaximumIterations(max_iter);
}

/**
 * @brief Ndt配准 源点云
 * @param[in]
 * @param[in]
 * @return
 */
void NdtRegistration::SetSourceCloud(const CloudMsg::CLOUD_PTR &source_cloud_ptr)
{
    ndt_ptr_->setInputSource(source_cloud_ptr);
}

/**
 * @brief Ndt配准 目标点云
 * @param[in]
 * @param[in]
 * @return
 */
void NdtRegistration::SetTargetCloud(const CloudMsg::CLOUD_PTR &target_cloud_ptr)
{
    ndt_ptr_->setInputTarget(target_cloud_ptr);
}

/**
 * @brief Ndt配准
 * @param[in]
 * @param[in]
 * @return
 */
bool NdtRegistration::Registration(const Eigen::Matrix4f &predict_pose, Eigen::Matrix4f &result_pose,
                                   CloudMsg::CLOUD_PTR &result_cloud_ptr)
{
    ndt_ptr_->align(*result_cloud_ptr, predict_pose);
    result_pose = ndt_ptr_->getFinalTransformation();
    return true;
}

} // namespace module