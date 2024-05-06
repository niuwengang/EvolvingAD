#include "path_pub.hpp"

namespace evolving_ad_ns
{
PathPub::PathPub(ros::NodeHandle &nh, const std::string topic_name, const std::string frame_id,
                 const size_t buffer_size)
{
    frame_id_ = frame_id;
    pub_ = nh.advertise<nav_msgs::Path>(topic_name, buffer_size);
}

void PathPub::Publish(const std::deque<Eigen::Isometry3d> &opted_pose_queue, const double time_stamp)
{
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = frame_id_;

    for (size_t i = 0; i < opted_pose_queue.size(); ++i)
    {
        Eigen::Isometry3d pose = opted_pose_queue.at(i);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = frame_id_;
        pose_stamped.header.seq = i;

        Eigen::Vector3d translation = pose.translation();
        pose_stamped.pose.position.x = translation.x();
        pose_stamped.pose.position.y = translation.y();
        pose_stamped.pose.position.z = translation.z();

        Eigen::Quaterniond q(pose.rotation());

        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        path.poses.push_back(pose_stamped);
    }

    pub_.publish(path);
}

} // namespace evolving_ad_ns
