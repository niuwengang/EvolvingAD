/**
 * @file    cloud_msg.cpp
 * @brief   点云消息封装
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 1.0
 */

#include "cloud_msg.hpp"

/**
 * @brief 点云消息初始化
 * @param[in] none
 * @return
 */
CloudMsg::CloudMsg()
{
    cloud_ptr.reset(new CLOUD()); // 分配内存
}