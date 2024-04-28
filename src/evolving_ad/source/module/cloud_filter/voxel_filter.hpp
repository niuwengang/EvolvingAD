/**
 * @file    cloud_filter.hpp
 * @brief    voxel filter
 * @author  niu_wengang@163.com
 * @date    2024-03-30
 * @version 0.1.0
 */

#include "cloud_filter_interface.hpp"
// pcl
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

namespace evolving_ad_ns
{
class VoxelFilter : public CloudFilterInterface
{
  public:
    VoxelFilter(const float leaf_size);

    bool Filter(const CloudMsg::CLOUD_PTR &source_cloud_ptr, CloudMsg::CLOUD_PTR &filtered_cloud_ptr) override;

  private:
    // pcl::ApproximateVoxelGrid<CloudMsg::POINT> voxel_filter_;
    pcl::VoxelGrid<CloudMsg::POINT> voxel_filter_;
};
} // namespace evolving_ad_ns
