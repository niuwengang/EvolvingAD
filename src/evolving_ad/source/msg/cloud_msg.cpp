/**
 * @file    cloud_msg.cpp
 * @brief   cloud msg
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 0.1.0
 */

#include "cloud_msg.hpp"

/**
 * @brief
 * @param[in] none
 * @return
 */
CloudMsg::CloudMsg()
{
    cloud_ptr.reset(new CLOUD());
}