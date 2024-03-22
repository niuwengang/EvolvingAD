/**
 * @file    cloud_sub.hpp
 * @brief   cloud_sub 激光点云订阅
 * @author  niu_wengang@163.com
 * @date    2024-03-09
 * @version 1.0
 * @ref https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html
 */

#ifndef _CLOUD_PUB_HPP_
#define _CLOUD_PUB_HPP_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class CloudPub
{
  public:
    CloudPub(ros::NodeHandle &nh, std::string topic_name, std::string frame_id, size_t buff_size);

    void Publish(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_ptr);

  private:
    ros::Publisher publisher_;
    std::string frame_id_;
};
#endif
