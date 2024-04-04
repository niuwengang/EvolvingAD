/**
 * @file    cloud_pub.cpp
 * @brief   点云消息发布
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 1.0
 */

#include "cloud_pub.hpp"

namespace Tools
{
/**
 * @brief 点云数据发布注册
 * @param[in]
 * @return
 */
CloudPub::CloudPub(ros::NodeHandle &nh, const std::string topic_name, const std::string frame_id,
                   const size_t buffer_size)

{
    frame_id_ = frame_id;
    pub_ = nh.advertise<sensor_msgs::PointCloud2>(topic_name, buffer_size);
}

/**
 * @brief 点云发布
 * @param[in]
 * @return
 */
void CloudPub::Publish(const CloudMsg::CLOUD_PTR &cloud_ptr)
{
    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_ptr, *cloud_ptr_output);
    cloud_ptr_output->header.stamp = ros::Time::now();
    cloud_ptr_output->header.frame_id = frame_id_;
    pub_.publish(*cloud_ptr_output);
}

/**
 * @brief 点云发布 重载
 * @param[in]
 * @return
 */
void CloudPub::Publish(const CloudMsg &cloud_msg)
{
    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_msg.cloud_ptr, *cloud_ptr_output);
    cloud_ptr_output->header.stamp = ros::Time(cloud_msg.time_stamp);
    cloud_ptr_output->header.frame_id = frame_id_;
    pub_.publish(*cloud_ptr_output);
}
} // namespace Tools