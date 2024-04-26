/**
 * @file    gnss_pub.cpp
 * @brief   gnss发布
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 1.0
 */

#include "gnss_pub.hpp"

namespace Tools
{
/**
 * @brief 点云数据发布注册
 * @param[in]
 * @return
 */
GnssPub::GnssPub(ros::NodeHandle &nh, const std::string topic_name, const std::string frame_id,
                 const size_t buffer_size)

{
    frame_id_ = frame_id;
    pub_ = nh.advertise<sensor_msgs::NavSatFix>(topic_name, buffer_size);
}

/**
 * @brief 点云发布
 * @param[in]
 * @return
 */
void GnssPub::Publish(const GnssMsg &gnss_msg)
{
    sensor_msgs::NavSatFix gnss_msg_ros;
    gnss_msg_ros.header.frame_id = frame_id_;
    gnss_msg_ros.header.stamp = ros::Time(gnss_msg.time_stamp);
    gnss_msg_ros.latitude = gnss_msg.latitude;
    gnss_msg_ros.longitude = gnss_msg.longitude;
    gnss_msg_ros.altitude = gnss_msg.altitude;

    pub_.publish(gnss_msg_ros);
}

} // namespace Tools