/**
 * @file    gnss_sub.cpp
 * @brief   gnss sub
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 0.1.0
 * @note
 * https://docs.ros.org/en/jade/api/sensor_msgs/html/msg/NavSatFix.html
 */

#include "gnss_sub.hpp"

namespace evolving_ad_ns
{
/**
 * @brief gnss subscriber init
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
 * @brief gnss message callback
 * @param[in] gnss_msg_ptr
 * @return
 */
void GnssSub::MsgCallback(const sensor_msgs::NavSatFixConstPtr &gnss_msg_ptr)
{
    mutex_.lock();
    /*[1]--load mesage*/
    GnssMsg gnss_msg;
    gnss_msg.time_stamp = ros::Time::now().toSec(); // gnss_msg_ptr->header.stamp.toSec();
    gnss_msg.latitude = gnss_msg_ptr->latitude;
    gnss_msg.longitude = gnss_msg_ptr->longitude;
    gnss_msg.altitude = gnss_msg_ptr->altitude;
    gnss_msg.status = gnss_msg_ptr->status.status;
    gnss_msg.service = gnss_msg_ptr->status.service;

    /*[2]--add msg to queue*/
    gnss_msg_queue_.push_back(gnss_msg);

    mutex_.unlock();
}

/**
 * @brief read gnss buffer
 * @param[in] gnss_msg_ptr
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
} // namespace evolving_ad_ns