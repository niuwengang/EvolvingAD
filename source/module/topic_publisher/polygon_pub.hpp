/**
 * @file    polygon_pub.hpp
 * @brief   polygon_pub 闭合多边形
 * @author  niu_wengang@163.com
 * @date    2024-03-09
 * @version 1.0
 * @ref https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html
 */

#include <jsk_recognition_msgs/PolygonArray.h>
#include <ros/ros.h>

class PolygonPub
{
  public:
    PolygonPub(ros::NodeHandle &nh, std::string topic_name, std::string frame_id, size_t buff_size);

    void Publish(const std::vector<std::array<float, 3>> &polygon_sequence); // 默认闭合

  private:
    ros::Publisher publisher_;
    std::string frame_id_;
};