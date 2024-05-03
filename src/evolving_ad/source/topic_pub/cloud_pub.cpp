/**
 * @file    cloud_pub.cpp
 * @brief   publish cloud
 * @author  niu_wengang@163.com
 * @date    2024-04-09 updae
 * @version 0.1.1
 */

#include "cloud_pub.hpp"

namespace evolving_ad_ns
{
/**
 * @brief
 * @param[in] nh
 * @param[in] topic_name
 * @param[in] frame_id
 * @param[in] buffer_size
 * @return
 */
CloudPub::CloudPub(ros::NodeHandle &nh, const std::string topic_name, const std::string frame_id,
                   const size_t buffer_size)

{
    frame_id_ = frame_id;
    pub_ = nh.advertise<sensor_msgs::PointCloud2>(topic_name, buffer_size);
}

/**
 * @brief publish (overloaded)
 * @param[in]
 * @return
 * @note use extern timestamp
 */
void CloudPub::Publish(const CloudMsg::CLOUD_PTR &cloud_ptr, const double time_stamp )
{

    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_ptr, *cloud_ptr_output);
    cloud_ptr_output->header.stamp = ros::Time(time_stamp);
    cloud_ptr_output->header.frame_id = frame_id_;

    pub_.publish(*cloud_ptr_output);
}

/**
 * @brief publish (overloaded)
 * @param[in] cloud_msg
 * @return
 * @note use interior timestamp
 */
void CloudPub::Publish(const CloudMsg &cloud_msg)
{
    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_msg.cloud_ptr, *cloud_ptr_output);
    cloud_ptr_output->header.stamp = ros::Time(cloud_msg.time_stamp);
    cloud_ptr_output->header.frame_id = frame_id_;

    pub_.publish(*cloud_ptr_output);
}
} // namespace evolving_ad_ns