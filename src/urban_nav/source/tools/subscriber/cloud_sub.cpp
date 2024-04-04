/**
 * @file    cloud_sub.cpp
 * @brief   点云消息订阅
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 1.0
 */

#include "cloud_sub.hpp"

namespace Tools
{
/**
 * @brief 点云订阅注册
 * @param[in] nh 句柄
 * @param[in] topic_name 话题名
 * @param[in] buffer_size 缓冲区大小
 * @return
 */
CloudSub::CloudSub(ros::NodeHandle &nh, const std::string topic_name, const size_t buffer_size)
{
    sub_ = nh.subscribe(topic_name, buffer_size, &CloudSub::MsgCallback, this);
}

/**
 * @brief 点云回调
 * @param[in] cloud_msg_ptr 点云指针
 * @return
 */
void CloudSub::MsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr)
{
    mutex_.lock();
    CloudMsg cloud_msg;
    cloud_msg.time_stamp = cloud_msg_ptr->header.stamp.toSec();
    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_msg.cloud_ptr));

    cloud_msg_queue_.push_back(cloud_msg);
    mutex_.unlock();
}

/**
 * @brief 读取点云序列
 * @param[in out] cloud_msg_queue 点云序列
 * @return
 */
void CloudSub::ParseData(std::deque<CloudMsg> &cloud_msg_queue)
{
    if (cloud_msg_queue_.size() > 0)
    {
        cloud_msg_queue.insert(cloud_msg_queue.end(), cloud_msg_queue_.begin(), cloud_msg_queue_.end());
        cloud_msg_queue_.clear();
    }
}
} // namespace Tools