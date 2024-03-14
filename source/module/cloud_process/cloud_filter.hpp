#ifndef _CLOUD_FILTER_HPP_
#define _CLOUD_FILTER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class CloudFilterInterface
{
  public:
    CloudFilterInterface()
    {
    }
    virtual void CloudFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out) = 0;
    virtual ~CloudFilterInterface() // 析构造必须为虚函数
    {
    }
};

class Voxel : virtual public CloudFilterInterface
{
  public:
    Voxel(const double leaf_size)
    {
        leaf_size_ = leaf_size;
    }
    void CloudFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out);

  private:
    double leaf_size_;
};

#endif //_CLOUD_FILTER_HPP_