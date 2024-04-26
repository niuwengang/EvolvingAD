/**
 * @file    odom_pub.cpp
 * @brief   publish odom
 * @author  niu_wengang@163.com
 * @date    2024-04-09
 * @version 0.1.1
 */

#include "odom_pub.hpp"

namespace Tools
{
/**
 * @brief odom pub init
 * @param[in] nh
 * @param[in] topic_name
 * @param[in] base_frame_id
 * @param[in] child_frame_id
 * @param[in] buffer_size
 * @return
 */
OdomPub::OdomPub(ros::NodeHandle &nh, const std::string topic_name, const std::string base_frame_id,
                 const std::string child_frame_id, const size_t buffer_size)

{

    pub_ = nh.advertise<nav_msgs::Odometry>(topic_name, buffer_size);

    odom_.header.frame_id = base_frame_id;
    odom_.child_frame_id = child_frame_id;
}
/**
 * @brief odom pub
 * @param[in] transform_matrix
 * @param[in] time_stamp
 * @return
 */
void OdomPub::Publish(const Eigen::Matrix4f &transform_matrix, const double time_stamp)
{
    odom_.header.stamp = ros::Time(time_stamp);

    // set the position
    odom_.pose.pose.position.x = transform_matrix(0, 3);
    odom_.pose.pose.position.y = transform_matrix(1, 3);
    odom_.pose.pose.position.z = transform_matrix(2, 3);

    Eigen::Quaternionf q;
    q = transform_matrix.block<3, 3>(0, 0);
    odom_.pose.pose.orientation.x = q.x();
    odom_.pose.pose.orientation.y = q.y();
    odom_.pose.pose.orientation.z = q.z();
    odom_.pose.pose.orientation.w = q.w();

    pub_.publish(odom_);
}

} // namespace Tools