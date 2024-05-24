/**
 * @file    cloud_msg.cpp
 * @brief   cloud msg
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 0.1.0
 */

#include "cloud_msg.hpp"
namespace evolving_ad_ns
{
/**
 * @brief
 * @param[in] none
 * @return
 */
CloudMsg::CloudMsg()
{
    cloud_ptr.reset(new CLOUD());
    // cloud_ptr->reserve(576000); // from https://www.robosense.cn/rslidar/RS-Helios
}

CloudMsg::CloudMsg(const CloudMsg &other)
{
    cloud_ptr.reset();
    this->time_stamp = other.time_stamp;
    this->cloud_ptr.reset(new CLOUD(*other.cloud_ptr)); // not release other
}

CloudMsg &CloudMsg::operator=(const CloudMsg &other)
{
    cloud_ptr.reset();
    this->time_stamp = other.time_stamp;
    this->cloud_ptr.reset(new CLOUD(*other.cloud_ptr)); // not release other
    return *this;
}

CloudMsg::~CloudMsg()
{
}

} // namespace evolving_ad_ns
