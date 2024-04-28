/**
 * @file    cloud_sub.cpp
 * @brief   cloud subscriber
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 0.1.1
 */

#include "cloud_sub.hpp"

namespace evolving_ad_ns
{
/**
 * @brief cloud subscriber
 * @param[in] nh handle
 * @param[in] topic_name
 * @param[in] buffer_size
 * @return
 */
CloudSub::CloudSub(ros::NodeHandle &nh, const std::string topic_name, const size_t buffer_size)
{
    sub_ = nh.subscribe(topic_name, buffer_size, &CloudSub::MsgCallback, this);
}

/**
 * @brief cloud subscriber callback
 * @param[in] cloud_msg_ptr
 * @return
 */
void CloudSub::MsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr)
{
    std::cout << "get lidar msg" << std::endl;
    mutex_.lock();

    /*[1]--load cloud message*/
    CloudMsg cloud_msg;
    cloud_msg.time_stamp = cloud_msg_ptr->header.stamp.toSec();
    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_msg.cloud_ptr));

    /*[2]--add message to queue*/
    cloud_msg_queue_.push_back(cloud_msg);

    mutex_.unlock();
}

/**
 * @brief read cloud buffer
 * @param[in out] cloud_msg_queue
 * @return
 */
void CloudSub::ParseMsg(std::deque<CloudMsg> &cloud_msg_queue)
{
    if (cloud_msg_queue_.size() > 0)
    {
        cloud_msg_queue.insert(cloud_msg_queue.end(), cloud_msg_queue_.begin(), cloud_msg_queue_.end());
        cloud_msg_queue_.clear();
    }
}
} // namespace evolving_ad_ns