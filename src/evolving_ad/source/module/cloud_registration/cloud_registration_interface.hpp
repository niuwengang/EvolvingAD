/**
 * @file    cloud_registration_interface.hpp
 * @brief   cloud_registration_interface
 * @author  niu_wengang@163.com
 * @date    2024-04-10
 * @version 0.1.1
 */

#ifndef _CLOUD_REGISTRATION_INTERFACE_HPP_
#define _CLOUD_REGISTRATION_INTERFACE_HPP_

// msg lib
#include "msg/cloud_msg.hpp"

namespace evolving_ad_ns
{

class CloudRegistrationInterface
{
  public:
    virtual ~CloudRegistrationInterface() = default; // 析构必须virtual

    virtual void SetSourceCloud(const CloudMsg::CLOUD_PTR &source_cloud_ptr) = 0;
    virtual void SetTargetCloud(const CloudMsg::CLOUD_PTR &target_cloud_ptr) = 0;
    virtual bool Registration(const Eigen::Matrix4f &predict_pose, Eigen::Matrix4f &result_pose,
                              CloudMsg::CLOUD_PTR &result_cloud_ptr) = 0;
};

} // namespace evolving_ad_ns

#endif