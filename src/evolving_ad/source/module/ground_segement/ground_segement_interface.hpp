/**
 * @file    ground_segement_interface.hpp
 * @brief   ground segement
 * @author  niu_wengang@163.com
 * @date    2024-04-23
 * @version 0.1.2
 */

#ifndef _GROUND_SEGEMENT_INTERFACE_HPP_
#define _GROUND_SEGEMENT_INTERFACE_HPP_

// msg lib
#include "msg/cloud_msg.hpp"

namespace evolving_ad_ns
{

class GroundSegementInterface
{
  public:
    virtual ~GroundSegementInterface() = default;
    virtual void Segement(const CloudMsg::CLOUD_PTR &cloud_in_ptr, CloudMsg::CLOUD_PTR &ground_cloud_ptr,
                          CloudMsg::CLOUD_PTR &no_ground_cloud_ptr) = 0;
};

} // namespace evolving_ad_ns

#endif //_GROUND_SEGEMENT_INTERFACE_HPP