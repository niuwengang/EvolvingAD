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
    ObjectMsg();
    void KfPredict();
    void KfUpdate(const ObjectMsg &detect);

    static float Quaternion2YawAngle(const Eigen::Quaternionf &q);
    static Eigen::Quaternionf YawAngle2Quaternion(const float yaw);

  public:
    float x = 0, y = 0, z = 0;       // center
    float w = 0, l = 0, h = 0;       // dim
    float v_x = 0, v_y = 0, v_z = 0; // speed

    Eigen::Quaternionf q; //  heading
    unsigned int id = 0;  // id is irrelevant to label
    float score = 0.0;    // choose more than 0.6
    Label label;          // od semantic class
    float lifetime = 1.0; // 0.0-1.0
    // kf param
    Eigen::Matrix<float, 6, 6> kf_F_matrix_ = Eigen::Matrix<float, 6, 6>::Identity();         // xyz vxvyvz
    Eigen::Matrix<float, 6, 6> kf_P_matrix_ = 0.1 * Eigen::Matrix<float, 6, 6>::Identity();   // xyz vxvyvz
    Eigen::Matrix<float, 6, 6> kf_Q_matrix_ = 0.001 * Eigen::Matrix<float, 6, 6>::Identity(); // xyz vxvyvz
    Eigen::Matrix<float, 3, 6> kf_H_matrix_ = Eigen::Matrix<float, 3, 6>::Zero();             // xyz vxvyvz
    Eigen::Matrix<float, 3, 3> kf_R_matrix_ = 0.1 * Eigen::Matrix<float, 3, 3>::Identity();   // xyz vxvyvz
    Eigen::Matrix<float, 6, 3> kf_K_matrix_ = Eigen::Matrix<float, 6, 3>::Zero();
};

class ObjectsMsg
{
  public:
    ObjectsMsg();
    ObjectsMsg(const ObjectsMsg &other);
    ObjectsMsg &operator=(const ObjectsMsg &other);
    ~ObjectsMsg();
    void TransCoord(const Eigen::Matrix4f relative_pose);

    double time_stamp = 0.0;
    std::vector<ObjectMsg> objects_vec;
};
} // namespace evolving_ad_ns
#endif //_OBJECT_MSG_HPP_