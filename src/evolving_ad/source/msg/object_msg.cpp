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
    objects_vec.reserve(50); // max is less than 50
}
} // namespace evolving_ad_ns