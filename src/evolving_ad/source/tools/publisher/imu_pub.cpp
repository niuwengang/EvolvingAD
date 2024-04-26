/**
 * @file    imu_pub.hpp
 * @brief   publish imu message
 * @author  niu_wengang@163.com
 * @date    2024-04-09 update
 * @version 0.1.1
 */

#include "imu_pub.hpp"

namespace Tools
{
/**
 * @brief imu publisher init
 * @param[in] nh
 * @param[in] topic_name
 * @param[in] frame_id
 * @param[in] buffer_size
 * @return
 */
ImuPub::ImuPub(ros::NodeHandle &nh, const std::string topic_name, const std::string frame_id, const size_t buffer_size)
{
    frame_id_ = frame_id;
    pub_ = nh.advertise<sensor_msgs::Imu>(topic_name, buffer_size);
}

/**
 * @brief publish imu message
 * @param[in] imu_msg
 * @return
 */
void ImuPub::Publish(const ImuMsg &imu_msg)
{
    /*[1]--fill data*/
    sensor_msgs::Imu imu_ros_msg;
    /*a--header*/
    imu_ros_msg.header.stamp = ros::Time(imu_msg.time_stamp);
    imu_ros_msg.header.frame_id = frame_id_;
    /*b--linear_acceleration*/
    imu_ros_msg.linear_acceleration.x = imu_msg.linear_acceleration.x;
    imu_ros_msg.linear_acceleration.y = imu_msg.linear_acceleration.y;
    imu_ros_msg.linear_acceleration.z = imu_msg.linear_acceleration.z;
    /*c--angular_velocity*/
    imu_ros_msg.angular_velocity.x = imu_msg.angular_velocity.x;
    imu_ros_msg.angular_velocity.y = imu_msg.angular_velocity.y;
    imu_ros_msg.angular_velocity.z = imu_msg.angular_velocity.z;
    /*d--orientation*/
    imu_ros_msg.orientation.x = imu_msg.orientation.x;
    imu_ros_msg.orientation.y = imu_msg.orientation.y;
    imu_ros_msg.orientation.z = imu_msg.orientation.z;
    imu_ros_msg.orientation.w = imu_msg.orientation.w;

    /*[2]--publish data*/
    pub_.publish(imu_ros_msg);

    return;
}
} // namespace Tools
