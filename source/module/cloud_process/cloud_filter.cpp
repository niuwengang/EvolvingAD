#include "cloud_filter.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

void Voxel::CloudFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in_ptr,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out_ptr)
{
    /*1--体素化*/
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_ptr(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(cloud_in_ptr);
    voxel_filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    voxel_filter.filter(*cloud_filtered_ptr);
    /*2--最近邻*/
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud_in_ptr);

    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    for (const auto &point : cloud_filtered_ptr->points)
    {
        if (kdtree.nearestKSearch(point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            auto point_update = point;
            // Update meaned intensity to original intensity
            point_update.intensity = (*cloud_in_ptr)[pointIdxNKNSearch[0]].intensity;
            cloud_intensity_ptr->points.push_back(point_update);
        }
    }
    *cloud_out_ptr = *cloud_intensity_ptr;
}