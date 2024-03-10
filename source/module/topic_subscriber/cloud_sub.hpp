/**
 * @file    cloud_sub.hpp
 * @brief   cloud_sub 激光点云订阅
 * @author  niu_wengang@163.com
 * @date    2024-03-09
 * @version 1.0
 * @ref https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html
 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class CloudSub
{
  public:
    CloudSub(ros::NodeHandle &nh, const std::string topic_name, const size_t buffer_size);

    void ParseData();

  private:
    void MsgCallback(const sensor_msgs::PointCloud2::ConstPtr &msg_ptr);

    ros::Subscriber cloud_sub_;
};
