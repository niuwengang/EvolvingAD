/**
 * @file    gnss_sub.cpp
 * @brief   imu消息订阅
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 1.0
 * @note
 * https://docs.ros.org/en/jade/api/sensor_msgs/html/msg/NavSatFix.html
 */

#include "gnss_sub.hpp"

namespace Tools
{
/**
 * @brief imu订阅回调
 * @param[in] nh
 * @param[in] topic_name
 * @param[in] buffer_size
 * @return
 */
GnssSub::GnssSub(ros::NodeHandle &nh, const std::string topic_name, const size_t buffer_size)
{
    sub_ = nh.subscribe(topic_name, buffer_size, &GnssSub::MsgCallback, this);
}

/**
 * @brief imu订阅回调
 * @param[in] gnss_msg_ptr gnss消息指针
 * @return
 */
void GnssSub::MsgCallback(const sensor_msgs::NavSatFixConstPtr &gnss_msg_ptr)
{
    mutex_.lock();
    GnssMsg gnss_msg;
    gnss_msg.time_stamp = gnss_msg_ptr->header.stamp.toSec();
    gnss_msg.latitude = gnss_msg_ptr->latitude;
    gnss_msg.longitude = gnss_msg_ptr->longitude;
    gnss_msg.altitude = gnss_msg_ptr->altitude;
    gnss_msg.status = gnss_msg_ptr->status.status;
    gnss_msg.service = gnss_msg_ptr->status.service;

    gnss_msg_queue_.push_back(gnss_msg);

    mutex_.unlock();
}

/**
 * @brief gnss读取
 * @param[in] gnss_msg_queue gnss消息队列
 * @return
 */
void GnssSub::ParseData(std::deque<GnssMsg> &gnss_msg_queue)
{
    if (gnss_msg_queue_.size() > 0)
    {
        gnss_msg_queue.insert(gnss_msg_queue.end(), gnss_msg_queue_.begin(), gnss_msg_queue_.end());
        gnss_msg_queue_.clear();
    }
}
} // namespace Tools