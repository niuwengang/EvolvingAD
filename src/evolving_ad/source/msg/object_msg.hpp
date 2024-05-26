/**
 * @file    od_msg.hpp
 * @brief   od message
 * @author  niu_wengang@163.com
 * @date    2024-04-11
 * @version 0.1.1
 * @note
 * jsk_recognition_msgs/BoundingBox.msg.
 * https://jsk-visualization.readthedocs.io/en/latest/jsk_rviz_plugins/plugins/bounding_box_array.html
 * https://jsk-visualization.readthedocs.io/en/latest/jsk_rviz_plugins/plugins/bounding_box.html
 */

#ifndef _OBJECT_MSG_HPP_
#define _OBJECT_MSG_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>
#include <string>

namespace evolving_ad_ns
{

class ObjectMsg
{
  public:
    enum class Label
    {
        Car,
        Pedestrian,
        Cyclist,
        Others
    };

  public:
    float x = 0, y = 0, z = 0; // center
    float w = 0, l = 0, h = 0; // dim
    Eigen::Quaternionf q;      //  heading
    unsigned int id = 0;       // id is irrelevant to label
    float score = 0.0;         // choose more than 0.6
    Label label;               // od semantic class
};

class ObjectsMsg
{
  public:
    ObjectsMsg();
    ObjectsMsg(const ObjectsMsg &other);
    ObjectsMsg &operator=(const ObjectsMsg &other);
    ~ObjectsMsg();

    double time_stamp = 0.0;
    std::vector<ObjectMsg> objects_vec;
};
} // namespace evolving_ad_ns
#endif //_OBJECT_MSG_HPP_