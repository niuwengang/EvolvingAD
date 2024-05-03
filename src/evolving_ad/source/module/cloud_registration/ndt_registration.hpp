/**
 * @file    ndt_registration.hpp
 * @brief   pcl NDT
 * @author  niu_wengang@163.com
 * @date    2024-03-30
 * @version 1.0
 */

#ifndef _NDT_REGISTRATION_HPP_
#define _NDT_REGISTRATION_HPP_

#include "cloud_registration_interface.hpp"
// pcl lib
#include <pcl/registration/ndt.h>

namespace evolving_ad_ns
{
class NdtRegistration : public CloudRegistrationInterface
{
  public:
    NdtRegistration(const float resolution, const float step_size, const float trans_eps, const int max_iter);
    void SetSourceCloud(const CloudMsg::CLOUD_PTR &source_cloud_ptr) override;
    void SetTargetCloud(const CloudMsg::CLOUD_PTR &target_cloud_ptr) override;
    bool Registration(const Eigen::Matrix4f &predict_pose, Eigen::Matrix4f &result_pose,
                      CloudMsg::CLOUD_PTR &result_cloud_ptr) override;

  private:
    pcl::NormalDistributionsTransform<CloudMsg::POINT, CloudMsg::POINT>::Ptr ndt_ptr_;
};
} // namespace evolving_ad_ns

#endif // NDT_REGISTRATION_HPP