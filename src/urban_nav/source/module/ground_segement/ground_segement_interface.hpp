/**
 * @file    ground_segement_interface.hpp
 * @brief   ground segement
 * @author  niu_wengang@163.com
 * @date    2024-04-23
 * @version 0.1.2
 */

#ifndef _GROUND_SEGEMENT_INTERFACE_HPP
#define _GROUND_SEGEMENT_INTERFACE_HPP

// msg lib
#include "user_msg/cloud_msg.hpp"

namespace Module
{

class GroundSegementInterface
{
  public:
    virtual ~GroundSegementInterface() = default; // must use virtual
    virtual void Segement(const CloudMsg::CLOUD_PTR &cloud_in_ptr, CloudMsg::CLOUD_PTR &ground_cloud_ptr,
                          CloudMsg::CLOUD_PTR &no_ground_cloud_ptr) = 0;
};

} // namespace Module

#endif //_GROUND_SEGEMENT_INTERFACE_HPP