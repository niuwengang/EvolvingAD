#include "polygon_pub.hpp"

PolygonPub::PolygonPub(ros::NodeHandle &nh, std::string topic_name, std::string frame_id, size_t buff_size)
{
    // nh.advertise<jsk_recognition_msgs::PolygonArray>("/SCDR/debug/map_marker", 100);
    frame_id_ = frame_id;
    nh.advertise<jsk_recognition_msgs::PolygonArray>(topic_name, buff_size);
}

void PolygonPub::Publish(const std::vector<std::vector<Eigen::Vector3d>> &polygon_array)
{
    polygon_array_ros_.header.stamp = ros::Time::now();
    polygon_array_ros_.header.frame_id = frame_id_;
    for (const auto polygon : polygon_array)
    {
        geometry_msgs::PolygonStamped polygon_ros;
        for (const auto point : polygon)
        {
            geometry_msgs::Point32 point_ros;
            point_ros.x = point.x();
            point_ros.y = point.y();
            point_ros.z = point.z();
            polygon_ros.polygon.points.push_back(point_ros);
        }
        polygon_array_ros_.polygons.push_back(polygon_ros);
    }
}