/**
 * @file    cloud_msg.hpp
 * @brief   cloud msg
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 0.1.0
 */

#ifndef _CLOUD_MSG_HPP_
#define _CLOUD_MSG_HPP_

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace evolving_ad_ns
{
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
} // namespace evolving_ad_ns

#endif //_CLOUD_MSG_HPP_