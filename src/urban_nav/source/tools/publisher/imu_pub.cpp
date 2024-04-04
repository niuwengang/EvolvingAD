/**
 * @file    imu_pub.hpp
 * @brief   imu消息发布
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 1.0
 */

#include "imu_pub.hpp"

namespace Tools
{
/**
 * @brief imu发布注册
 * @param[in]
 * @return
 */
ImuPub::ImuPub(ros::NodeHandle &nh, const std::string topic_name, const std::string frame_id, const size_t buffer_size)
{
    frame_id_ = frame_id;
    pub_ = nh.advertise<sensor_msgs::Imu>(topic_name, buffer_size);
}
/**
 * @brief imu发布数据
 * @param[in]
 * @return
 */
void ImuPub::Publish(const ImuMsg &imu_msg)
{
    sensor_msgs::Imu imu_ros_msg;

    // 填充IMU消息头
    imu_ros_msg.header.stamp = ros::Time(imu_msg.time_stamp);
    imu_ros_msg.header.frame_id = frame_id_;
    // 加速度
    imu_ros_msg.linear_acceleration.x = imu_msg.linear_acceleration.x;
    imu_ros_msg.linear_acceleration.y = imu_msg.linear_acceleration.y;
    imu_ros_msg.linear_acceleration.z = imu_msg.linear_acceleration.z;
    // 角速度
    imu_ros_msg.angular_velocity.x = imu_msg.angular_velocity.x;
    imu_ros_msg.angular_velocity.y = imu_msg.angular_velocity.y;
    imu_ros_msg.angular_velocity.z = imu_msg.angular_velocity.z;
    // 四元数
    imu_ros_msg.orientation.x = imu_msg.orientation.x;
    imu_ros_msg.orientation.y = imu_msg.orientation.y;
    imu_ros_msg.orientation.z = imu_msg.orientation.z;
    imu_ros_msg.orientation.w = imu_msg.orientation.w;
    // 发布
    pub_.publish(imu_ros_msg);
}
} // namespace Tools
