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
    pub_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(topic_name, buffer_size);
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

        tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, object_msg.heading);
        od_bbox.pose.orientation.x = q.x();
        od_bbox.pose.orientation.y = q.y();
        od_bbox.pose.orientation.z = q.z();
        od_bbox.pose.orientation.w = q.w();
        od_bbox.dimensions.x = object_msg.w; //! may have issue
        od_bbox.dimensions.y = object_msg.l;
        od_bbox.dimensions.z = object_msg.h;
        od_bbox.value = object_msg.score;

        ods_bbox.boxes.push_back(od_bbox);
    }

    pub_.publish(ods_bbox);
}

} // namespace evolving_ad_ns