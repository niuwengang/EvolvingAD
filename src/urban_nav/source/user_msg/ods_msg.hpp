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

#include <Eigen/Core>
#include <deque>
#include <string>

#ifndef _ODS_HPP_
#define _ODS_HPP_

class OdMsg
{
    enum class Label
    {
        Car,
        Pedestrian,
        Cyclist,
        Others
    };

  public:
    float x, y, z; // center
    float w, l, h; // dim
    float heading;
    int id;
    float score;
    Label label; // od semantic class
    double time_stamp = 0.0;
};

class OdsMsg
{
  public:
    double time_stamp = 0.0;
    std::deque<OdMsg> ods_queue;
};

#endif