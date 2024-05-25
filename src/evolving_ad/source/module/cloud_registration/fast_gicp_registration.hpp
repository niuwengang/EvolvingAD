/**
 * @file    ndt_registration.hpp
 * @brief   pcl NDT
 * @author  niu_wengang@163.com
 * @date    2024-03-30
 * @version 1.0
 */
#ifndef _FAST_GICP_REGISTRATION_HPP
#define _FAST_GICP_REGISTRATION_HPP

#include "cloud_registration_interface.hpp"
// pcl
#include "thirdpartylib/fast_gicp/include/fast_gicp.hpp"
#include "thirdpartylib/fast_gicp/include/fast_vgicp.hpp"

namespace evolving_ad_ns
{
class FastGicpRegistration : public CloudRegistrationInterface
{
  public:
    FastGicpRegistration(const float resolution, const float step_size = 0, const float trans_eps = 0,
                         const int max_iter = 0);
    void SetSourceCloud(const CloudMsg::CLOUD_PTR &source_cloud_ptr) override;
    void SetTargetCloud(const CloudMsg::CLOUD_PTR &target_cloud_ptr) override;
    bool Registration(const Eigen::Matrix4f &predict_pose, Eigen::Matrix4f &result_pose,
                      CloudMsg::CLOUD_PTR &result_cloud_ptr) override;

  private:
    fast_gicp::FastVGICP<CloudMsg::POINT, CloudMsg::POINT> gicp_; // vgicp
    // fast_gicp::FastGICP<CloudMsg::POINT, CloudMsg::POINT> gicp_; // 多线程gicp
};
} // namespace evolving_ad_ns

#endif // NDT_REGISTRATION_HPP