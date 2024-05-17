/**
 * @file    bbx_pub.cpp
 * @brief   publish box
 * @author  niu_wengang@163.com
 * @date    2024-04-12 updae
 * @version 0.1.1
 */

#include "bbx_pub.hpp"

namespace evolving_ad_ns
{
/**
 * @brief
 * @param[in] nh
 * @param[in] topic_name
 * @param[in] frame_id
 * @param[in] buffer_size
 * @return
 */
BbxPub::BbxPub(ros::NodeHandle &nh, const std::string topic_name, const std::string frame_id, const size_t buffer_size)
{
    frame_id_ = frame_id;
    bbx_pub_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(topic_name, buffer_size);
    marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("bbx_str", 100);
}

/**
 * @brief publish (overloaded)
 * @param[in]
 * @return
 * @note use extern timestamp
 */
void BbxPub::Publish(const ObjectsMsg &objects_msg)
{
    jsk_recognition_msgs::BoundingBoxArray ods_bbox;

    ods_bbox.header.frame_id = "map";
    ods_bbox.header.stamp = ros::Time(objects_msg.time_stamp);
    for (auto object_msg : objects_msg.objects_vec)
    {
        jsk_recognition_msgs::BoundingBox od_bbox;

        od_bbox.header.stamp = ods_bbox.header.stamp;

        od_bbox.header.frame_id = "map";
        od_bbox.pose.position.x = object_msg.x;
        od_bbox.pose.position.y = object_msg.y;
        od_bbox.pose.position.z = object_msg.z;

        od_bbox.pose.orientation.x = object_msg.q.x();
        od_bbox.pose.orientation.y = object_msg.q.y();
        od_bbox.pose.orientation.z = object_msg.q.z();
        od_bbox.pose.orientation.w = object_msg.q.w();
        od_bbox.dimensions.x = object_msg.w; //! may have issue
        od_bbox.dimensions.y = object_msg.l;
        od_bbox.dimensions.z = object_msg.h;
        od_bbox.value = object_msg.score;

        ods_bbox.boxes.push_back(od_bbox);
    }
    bbx_pub_.publish(ods_bbox);

    visualization_msgs::MarkerArray text_marker_array;
    for (int index = 0; index < 100; index++)
    {
        visualization_msgs::Marker text_marker;

        text_marker.header.frame_id = "map";
        text_marker.header.stamp = ros::Time::now();

        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.ns = "basic_shapes";
        text_marker.pose.orientation.w = 1.0;
        text_marker.id = index;

        text_marker.scale.x = 1.5;
        text_marker.scale.y = 1.5;
        text_marker.scale.z = 1.5;

        text_marker.color.b = 25;
        text_marker.color.g = 0;
        text_marker.color.r = 25;
        text_marker.color.a = 0; // same as delete

        geometry_msgs::Pose pose;
        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = 0;
        text_marker.pose = pose;

        text_marker.text = std::to_string(index);
        text_marker_array.markers.push_back(text_marker);
    }

    int index = 0;
    for (auto object_msg : objects_msg.objects_vec)
    {
        geometry_msgs::Pose pose;
        pose.position.x = object_msg.x;
        pose.position.y = object_msg.y;
        pose.position.z = object_msg.z;

        text_marker_array.markers[index].pose = pose;
        text_marker_array.markers[index].color.a = 1;
        index++;
    }
    marker_pub_.publish(text_marker_array);
}

} // namespace evolving_ad_ns