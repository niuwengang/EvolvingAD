#include "cloud_pub.hpp"
#include <pcl_conversions/pcl_conversions.h>

CloudPub::CloudPub(ros::NodeHandle &nh, std::string topic_name, std::string frame_id, size_t buff_size)
{
    frame_id_ = frame_id;
    publisher_ = nh.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
}

void CloudPub::Publish(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_ptr)
{
    sensor_msgs::PointCloud2Ptr cloud_ros_ptr(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_in_ptr, *cloud_ros_ptr);

    cloud_ros_ptr->header.stamp = ros::Time::now();
    cloud_ros_ptr->header.frame_id = frame_id_;
    publisher_.publish(*cloud_ros_ptr);
}