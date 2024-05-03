/**
 * @file    tf_pub.hpp
 * @brief   tf pub
 * @author  niu_wengang@163.com
 * @date    2024-03-30
 * @version 0.1.0
 */

#include "tf_pub.hpp"

namespace evolving_ad_ns
{
/**
 * @brief odom init
 * @param[in]
 * @return
 */
TfPub::TfPub(const std::string frame_id, const std::string child_frame_id)
{
    transform_.frame_id_ = frame_id;
    transform_.child_frame_id_ = child_frame_id;
}
/**
 * @brief tf pub
 * @param[in]
 * @return
 */
void TfPub::SendTransform(const Eigen::Matrix4f pose)
{
    Eigen::Quaternionf q(pose.block<3, 3>(0, 0));
    transform_.stamp_ = ros::Time::now();
    transform_.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    transform_.setOrigin(tf::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
    broadcaster_.sendTransform(transform_);
}
} // namespace evolving_ad_ns