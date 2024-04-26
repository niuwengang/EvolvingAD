/**
 * @file    voxel_filter.hpp
 * @brief   体素滤波器
 * @author  niu_wengang@163.com
 * @date    2024-03-30
 * @version 1.0
 */

#include "voxel_filter.hpp"

namespace Module
{

/**
 * @brief 体素滤波器
 * @param[in] leaf_size 滤波器尺寸
 * @return
 */
VoxelFilter::VoxelFilter(const float leaf_size)
{
    voxel_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);
}

/**
 * @brief 体素滤波器
 * @param[in] input_cloud_ptr 原始点云
 * @param[in] filtered_cloud_ptr 滤波后的点云
 * @return
 */
bool VoxelFilter::Filter(const CloudMsg::CLOUD_PTR &input_cloud_ptr, CloudMsg::CLOUD_PTR &filtered_cloud_ptr)
{
    voxel_filter_.setInputCloud(input_cloud_ptr);
    voxel_filter_.filter(*filtered_cloud_ptr);

    return true;
}
} // namespace Module
