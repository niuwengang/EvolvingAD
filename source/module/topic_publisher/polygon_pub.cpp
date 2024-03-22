#include "polygon_pub.hpp"
#include "Eigen/Core"

PolygonPub::PolygonPub(ros::NodeHandle &nh, std::string topic_name, std::string frame_id, size_t buff_size)
{
}

void PolygonPub::Publish(const std::vector < Egen::Vector3d >> &block_sequence)
{
    /*队列顺序*/

    for (const auto it : block_sequence)
    {
        geometry_msgs::PolygonStamped polygon;

        geometry_msgs::Point32 point;
        point.x =
    }

    jsk_recognition_msgs::PolygonArray polygon_sequence;

    polygon_sequence.polygons.push_back(polygon);
}