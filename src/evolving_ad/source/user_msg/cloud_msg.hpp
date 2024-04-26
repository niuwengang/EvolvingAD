/**
 * @file    cloud_msg.hpp
 * @brief   点云消息封装
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 1.0
 */

#ifndef CLOUD_MSG_HPP
#define CLOUD_MSG_HPP

// pcl lib
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class CloudMsg
{

  public:
    CloudMsg();
    ~CloudMsg() = default;

    using POINT = pcl::PointXYZ;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;

    double time_stamp = 0.0;
    CLOUD_PTR cloud_ptr;
};

#endif