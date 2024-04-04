/**
 * @file    imu_sub.cpp
 * @brief   imu消息订阅
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 1.0
 */

#include "imu_sub.hpp"

namespace Tools
{
/**
 * @brief imu订阅初始化
 * @param[in] nh 句柄
 * @param[in] topic_name 话题名
 * @param[in] buffer_size 缓冲区大小
 * @return
 */
ImuSub::ImuSub(ros::NodeHandle &nh, const std::string topic_name, const size_t buffer_size)
{
    sub_ = nh.subscribe(topic_name, buffer_size, &ImuSub::MsgCallback, this);
}

/**
 * @brief imu订阅回调
 * @param[in] imu_msg_ptr imu消息指针
 * @return
 */
void ImuSub::MsgCallback(const sensor_msgs::ImuConstPtr &imu_msg_ptr)
{
    mutex_.lock();
    ImuMsg imu_msg;
    imu_msg.time_stamp = imu_msg_ptr->header.stamp.toSec();

    imu_msg.linear_acceleration.x = imu_msg_ptr->linear_acceleration.x;
    imu_msg.linear_acceleration.y = imu_msg_ptr->linear_acceleration.y;
    imu_msg.linear_acceleration.z = imu_msg_ptr->linear_acceleration.z;

    imu_msg.angular_velocity.x = imu_msg_ptr->angular_velocity.x;
    imu_msg.angular_velocity.y = imu_msg_ptr->angular_velocity.y;
    imu_msg.angular_velocity.z = imu_msg_ptr->angular_velocity.z;

    imu_msg.orientation.x = imu_msg_ptr->orientation.x;
    imu_msg.orientation.y = imu_msg_ptr->orientation.y;
    imu_msg.orientation.z = imu_msg_ptr->orientation.z;
    imu_msg.orientation.w = imu_msg_ptr->orientation.w;

    imu_msg_queue_.push_back(imu_msg);
    mutex_.unlock();
}

/**
 * @brief imu读取
 * @param[in out] imu_msg_queue imu消息队列
 * @return
 */
void ImuSub::ParseData(std::deque<ImuMsg> &imu_msg_queue)
{
    if (imu_msg_queue_.size() > 0)
    {
        imu_msg_queue.insert(imu_msg_queue.end(), imu_msg_queue_.begin(), imu_msg_queue_.end());
        imu_msg_queue_.clear();
    }
}
} // namespace Tools