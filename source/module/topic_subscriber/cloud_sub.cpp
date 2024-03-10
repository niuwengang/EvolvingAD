/**
 * @file    cloud_sub.cpp
 * @brief   cloud_sub 激光点云订阅
 * @author  niu_wengang@163.com
 * @date    2024-03-09
 * @version 1.0
 * @ref https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html
 */

#include "cloud_sub.hpp"
#include <pcl_ros/point_cloud.h>

/**
 * @brief 激光雷达订阅
 * @param[in] a - 第一个加数
 * @param[in] b - 第二个加数
 * @return 两个加数的和
 */
CloudSub::CloudSub(ros::NodeHandle &nh, const std::string topic_name, const size_t buffer_size)
{

    cloud_sub_ = nh.subscribe(topic_name, buffer_size, &CloudSub::MsgCallback, this);
}

void CloudSub::ParseData()
{
}

void CloudSub::MsgCallback(const sensor_msgs::PointCloud2::ConstPtr &msg_ptr)
{
    std::cout << "接收.." << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg_ptr, *cloud_ptr);
    std::cout << "点云数量:" << cloud_ptr->points.size() << std::endl;
}
