/**
 * @file    bbx_pub.cpp
 * @brief   publish box
 * @author  niu_wengang@163.com
 * @date    2024-04-12 updae
 * @version 0.1.1
 */

#include "bbx_pub.hpp"

namespace Tools
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
    pub_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(topic_name, buffer_size);
}

/**
 * @brief publish (overloaded)
 * @param[in]
 * @return
 * @note use extern timestamp
 */
void BbxPub::Publish(const OdsMsg &ods_msg)
{
    jsk_recognition_msgs::BoundingBoxArray ods_bbox;

    ods_bbox.header.frame_id = "map";
    ods_bbox.header.stamp = ros::Time(ods_msg.time_stamp);
    for (auto od_msg : ods_msg.ods_queue)
    {
        jsk_recognition_msgs::BoundingBox od_bbox;

        od_bbox.header.stamp = ros::Time(ods_msg.time_stamp);
        od_bbox.header.frame_id = "map";
        od_bbox.pose.position.x = od_msg.pos(0);
        od_bbox.pose.position.y = od_msg.pos(1);
        od_bbox.pose.position.z = od_msg.pos(2);
        od_bbox.pose.orientation.x = od_msg.orientation(0);
        od_bbox.pose.orientation.y = od_msg.orientation(1);
        od_bbox.pose.orientation.z = od_msg.orientation(2);
        od_bbox.pose.orientation.w = od_msg.orientation(3);
        od_bbox.dimensions.x = od_msg.dim(0);
        od_bbox.dimensions.y = od_msg.dim(1);
        od_bbox.dimensions.z = od_msg.dim(2);

        ods_bbox.boxes.push_back(od_bbox);
    }

    pub_.publish(ods_bbox);
}

} // namespace Tools