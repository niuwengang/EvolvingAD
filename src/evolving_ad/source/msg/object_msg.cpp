/**
 * @file    od_msg.cpp
 * @brief   od message
 * @author  niu_wengang@163.com
 * @date    2024-04-11
 * @version 0.1.1
 * @note
 * jsk_recognition_msgs/BoundingBox.msg.
 * https://jsk-visualization.readthedocs.io/en/latest/jsk_rviz_plugins/plugins/bounding_box_array.html
 * https://jsk-visualization.readthedocs.io/en/latest/jsk_rviz_plugins/plugins/bounding_box.html
 */

#include "object_msg.hpp"

namespace evolving_ad_ns
{

ObjectsMsg::ObjectsMsg()
{
    objects_vec.reserve(100); // max is less than 100
}

ObjectsMsg &ObjectsMsg::operator=(const ObjectsMsg &other)
{
    this->time_stamp = other.time_stamp;   // timestamp
    this->objects_vec = other.objects_vec; // objects_vec
    return *this;
}

ObjectsMsg::ObjectsMsg(const ObjectsMsg &other)
{
    this->time_stamp = other.time_stamp;   // timestamp
    this->objects_vec = other.objects_vec; // objects_vec
}

void ObjectsMsg::TransCoord(const Eigen::Matrix4f relative_pose)
{
    for (auto &object : this->objects_vec)
    {
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block<3, 1>(0, 3) << object.x, object.y, object.z; // t
        pose.block<3, 3>(0, 0) = object.q.toRotationMatrix();   // R

        pose = relative_pose * pose;

        object.x = pose(0, 3);
        object.y = pose(1, 3);
        object.z = pose(2, 3);
        object.q = Eigen::Quaternionf(pose.block<3, 3>(0, 0));
    }
}

ObjectsMsg::~ObjectsMsg()
{
}
} // namespace evolving_ad_ns