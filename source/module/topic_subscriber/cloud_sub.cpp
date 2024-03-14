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
 * @brief 点云订阅注册函数
 * @param[in] nh ros句柄
 * @param[in] topic_name 话题名
 * @param[in] buffer_size 缓冲区大小
 * @return none
 */
CloudSub::CloudSub(ros::NodeHandle &nh, const std::string topic_name, const size_t buffer_size)
{
    cloud_sub_ = nh.subscribe(topic_name, buffer_size, &CloudSub::MsgCallback, this);
}

void CloudSub::ParseData()
{
}

/**
 * @brief 点云订阅回调函数
 * @param[in] msg_ptr ros句柄
 * @param[in] topic_name 话题名
 * @param[in] buffer_size 缓冲区大小
 * @return none
 */
void CloudSub::MsgCallback(const sensor_msgs::PointCloud2::ConstPtr &msg_ptr)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg_ptr, *cloud_ptr);
    std::cout << "已接收到点云数量:" << cloud_ptr->points.size() << std::endl;
}
