/**
 * @file    voxel_filter.hpp
 * @brief   voxel filter
 * @author  niu_wengang@163.com
 * @date    2024-03-30
 * @version 0.1.0
 */

#include "voxel_filter.hpp"

namespace evolving_ad_ns
{

/**
 * @brief
 * @param[in] leaf_size
 * @return
 */
VoxelFilter::VoxelFilter(const float leaf_size)
{
    voxel_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);
}

/**
 * @brief
 * @param[in] input_cloud_ptr
 * @param[in] filtered_cloud_ptr
 * @return
 */
bool VoxelFilter::Filter(const CloudMsg::CLOUD_PTR &input_cloud_ptr, CloudMsg::CLOUD_PTR &filtered_cloud_ptr)
{
    voxel_filter_.setInputCloud(input_cloud_ptr);
    voxel_filter_.filter(*filtered_cloud_ptr);

    return true;
}
} // namespace evolving_ad_ns
