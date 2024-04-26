/**
 * @file    imu_sub.cpp
 * @brief  subscribe imu
 * @author  niu_wengang@163.com
 * @date    2024-04-09
 * @version 0.1.1
 */

#include "imu_sub.hpp"

namespace Tools
{
/**
 * @brief imu subscriber init
 * @param[in] nh handle
 * @param[in] topic_name
 * @param[in] buffer_size
 * @return
 */
ImuSub::ImuSub(ros::NodeHandle &nh, const std::string topic_name, const size_t buffer_size)
{
    sub_ = nh.subscribe(topic_name, buffer_size, &ImuSub::MsgCallback, this);
}

/**
 * @brief imu callback
 * @param[in] imu_msg_ptr
 * @return
 */
void ImuSub::MsgCallback(const sensor_msgs::ImuConstPtr &imu_msg_ptr)
{
    mutex_.lock();

    /*[1]--load information */
    ImuMsg imu_msg;
    /*a--timestamp */
    imu_msg.time_stamp = imu_msg_ptr->header.stamp.toSec();
    /*b--linear_acceleration */
    imu_msg.linear_acceleration.x = imu_msg_ptr->linear_acceleration.x;
    imu_msg.linear_acceleration.y = imu_msg_ptr->linear_acceleration.y;
    imu_msg.linear_acceleration.z = imu_msg_ptr->linear_acceleration.z;
    /*c--angular_velocity */
    imu_msg.angular_velocity.x = imu_msg_ptr->angular_velocity.x;
    imu_msg.angular_velocity.y = imu_msg_ptr->angular_velocity.y;
    imu_msg.angular_velocity.z = imu_msg_ptr->angular_velocity.z;
    /*d--orientation*/
    imu_msg.orientation.x = imu_msg_ptr->orientation.x;
    imu_msg.orientation.y = imu_msg_ptr->orientation.y;
    imu_msg.orientation.z = imu_msg_ptr->orientation.z;
    imu_msg.orientation.w = imu_msg_ptr->orientation.w;

    /*[2]--add to message queue*/
    imu_msg_queue_.push_back(imu_msg);

    mutex_.unlock();

    return;
}

/**
 * @brief read imu buffer
 * @param[in out] imu_msg_queue
 * @return
 */
void ImuSub::ParseData(std::deque<ImuMsg> &imu_msg_queue)
{
    if (imu_msg_queue_.size() > 0)
    {
        imu_msg_queue.insert(imu_msg_queue.end(), imu_msg_queue_.begin(), imu_msg_queue_.end());
        imu_msg_queue_.clear();
    }
    return;
}
} // namespace Tools