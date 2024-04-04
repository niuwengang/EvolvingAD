/**
 * @file    cloud_sub.hpp
 * @brief   点云消息订阅
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 1.0
 */

// msg lib
#include "user_msg/cloud_msg.hpp"
// c++ lib
#include <deque>
#include <mutex>
// pcl lib
#include <pcl_conversions/pcl_conversions.h>
// ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace Tools
{

class CloudSub
{
  public:
    CloudSub() = default;
    CloudSub(ros::NodeHandle &nh, const std::string topic_name, const size_t buffer_size = 1000000);
    ~CloudSub() = default;
    void ParseData(std::deque<CloudMsg> &cloud_msg_queue);

  private:
    void MsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::mutex mutex_;

    std::deque<CloudMsg> cloud_msg_queue_;
};
} // namespace Tools