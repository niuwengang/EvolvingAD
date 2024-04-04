/**
 * @file    cloud_registration_interface.hpp
 * @brief   点云滤波虚接口cloud_registration_interface
 * @author  niu_wengang@163.com
 * @date    2024-03-30
 * @version 1.0
 */

#ifndef CLOUD_REGISTRATION_INTERFACE_HPP
#define CLOUD_REGISTRATION_INTERFACE_HPP

// msg lib
#include "user_msg/cloud_msg.hpp"

namespace Module
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

} // namespace module

#endif