/**
 * @file    cloud_filter_interface.hpp
 * @brief   cloud filter
 * @author  niu_wengang@163.com
 * @date    2024-03-30
 * @version 0.1.0
 */

#ifndef _CLOUD_FILTER_INTERFACE_HPP
#define _CLOUD_FILTER_INTERFACE_HPP

#include "msg/cloud_msg.hpp"

namespace evolving_ad_ns
{
class CloudFilterInterface
{
  public:
    virtual ~CloudFilterInterface() = default;
    virtual bool Filter(const CloudMsg::CLOUD_PTR &source_cloud_ptr, CloudMsg::CLOUD_PTR &filtered_cloud_ptr) = 0;
};
} // namespace evolving_ad_ns

#endif