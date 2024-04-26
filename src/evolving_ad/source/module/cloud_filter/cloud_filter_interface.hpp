/**
 * @file    cloud_filter_interface.hpp
 * @brief   点云滤波虚接口
 * @author  niu_wengang@163.com
 * @date    2024-03-30
 * @version 1.0
 */

#ifndef CLOUD_FILTER_INTERFACE_HPP
#define CLOUD_FILTER_INTERFACE_HPP

#include "user_msg/cloud_msg.hpp"

namespace Module
{
class CloudFilterInterface
{
  public:
    virtual ~CloudFilterInterface() = default;
    virtual bool Filter(const CloudMsg::CLOUD_PTR &source_cloud_ptr, CloudMsg::CLOUD_PTR &filtered_cloud_ptr) = 0;
};
} // namespace Module

#endif