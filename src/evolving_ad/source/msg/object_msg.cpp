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

ObjectMsg::ObjectMsg()
{
    lifetime = 1.0;
    const float dt = 0.1; // 100ms
    kf_F_matrix_(0, 3) = dt;
    kf_F_matrix_(1, 4) = dt;
    kf_F_matrix_(2, 5) = dt;

    kf_H_matrix_.diagonal().setOnes();
}

void ObjectMsg::KfPredict()
{
    lifetime -= 0.1;
    Eigen::Matrix<float, 6, 1> state_est;
    state_est << this->x, this->y, this->z, this->v_x, this->v_y, this->v_z; // dim is 6
    state_est = this->kf_F_matrix_ * state_est;                              // update state

    this->x = state_est[0];
    this->y = state_est[1];
    this->z = state_est[2];
    this->v_x = state_est[3];
    this->v_y = state_est[4];
    this->v_z = state_est[5];

    this->kf_P_matrix_ = kf_F_matrix_ * kf_P_matrix_ * kf_F_matrix_.transpose() + kf_Q_matrix_;
}
void ObjectMsg::KfUpdate(const ObjectMsg &detect)
{
    lifetime += 0.1;

    Eigen::Matrix<float, 6, 1> state_est;
    state_est << this->x, this->y, this->z, this->v_x, this->v_y, this->v_z; // dim is 6

    Eigen::Matrix<float, 3, 1> state_detect;
    state_detect << detect.x, detect.y, detect.z;

    state_est = state_est + kf_K_matrix_ * (state_detect - kf_H_matrix_ * state_est); // error
    kf_P_matrix_ = kf_P_matrix_ - kf_K_matrix_ * kf_H_matrix_ * kf_P_matrix_;
    kf_K_matrix_ = kf_P_matrix_ * kf_H_matrix_.transpose() *
                   (kf_H_matrix_ * kf_P_matrix_ * kf_H_matrix_.transpose() + kf_R_matrix_).inverse();
}
/*-------------------------------------------------------------------*/

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

// objects_msg_esti_.objects_vec.clear();
// for (unsigned int index = 0; index < objects_msg_prev_.objects_vec.size(); index++)
// {
//     auto object = objects_msg_prev_.objects_vec[index]; // cpoy one

//     objects_msg_esti_.objects_vec.emplace_back(object);
// }
