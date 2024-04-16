/**
 * @file    cloud_sub.cpp
 * @brief   里程计订阅
 * @author  niu_wengang@163.com
 * @date    2024-04-06
 * @version 1.0
 */

#include "odom_sub.hpp"

namespace Tools
{

/**
 * @brief 注册函数
 * @param[in out] imu_msg_queue pose消息队列
 * @return
 */
OdomSub::OdomSub(ros::NodeHandle &nh, const std::string topic_name, const size_t buffer_size)
{
    sub_ = nh.subscribe(topic_name, buffer_size, &OdomSub::MsgCallback, this);
}

/**
 * @brief 回调函数
 * @param[in out] imu_msg_queue pose消息队列
 * @return
 */
void OdomSub::MsgCallback(const nav_msgs::OdometryConstPtr &odom_msg_ptr)
{
    mutex_.lock();
    PoseMsg pose_msg;
    pose_msg.time_stamp = odom_msg_ptr->header.stamp.toSec();

    // set the position
    pose_msg.pose(0, 3) = odom_msg_ptr->pose.pose.position.x;
    pose_msg.pose(1, 3) = odom_msg_ptr->pose.pose.position.y;
    pose_msg.pose(2, 3) = odom_msg_ptr->pose.pose.position.z;

    Eigen::Quaternionf q;
    q.x() = odom_msg_ptr->pose.pose.orientation.x;
    q.y() = odom_msg_ptr->pose.pose.orientation.y;
    q.z() = odom_msg_ptr->pose.pose.orientation.z;
    q.w() = odom_msg_ptr->pose.pose.orientation.w;
    pose_msg.pose.block<3, 3>(0, 0) = q.matrix();

    pose_msg_queue_.push_back(pose_msg);
    mutex_.unlock();
}

/**
 * @brief imu读取
 * @param[in out] imu_msg_queue pose消息队列
 * @return
 */
void OdomSub::ParseData(std::deque<PoseMsg> &pose_msg_queue)
{
    if (pose_msg_queue_.size() > 0)
    {
        pose_msg_queue.insert(pose_msg_queue.end(), pose_msg_queue_.begin(), pose_msg_queue_.end());
        pose_msg_queue_.clear();
    }
}
}