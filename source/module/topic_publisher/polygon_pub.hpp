/**
 * @file    polygon_pub.hpp
 * @brief   polygon_pub 闭合多边形
 * @author  niu_wengang@163.com
 * @date    2024-03-09
 * @version 1.0
 * @ref https://docs.ros.org/en/indigo/api/jsk_recognition_msgs/html/msg/PolygonArray.html
 */

#include <Eigen/Core>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <ros/ros.h>

class PolygonPub
{
  public:
    PolygonPub(ros::NodeHandle &nh, std::string topic_name, std::string frame_id, size_t buff_size);

    void Publish(const std::vector<std::vector<Eigen::Vector3d>> &polygon_array); // 默认闭合polygon

  private:
    ros::Publisher publisher_;
    std::string frame_id_;
    jsk_recognition_msgs::PolygonArray polygon_array_ros_;
};