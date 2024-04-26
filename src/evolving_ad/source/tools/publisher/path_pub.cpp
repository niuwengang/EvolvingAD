#include "path_pub.hpp"

namespace Tools
{
PathPub::PathPub(ros::NodeHandle &nh, const std::string topic_name, const std::string frame_id,
                 const size_t buffer_size)
{
    frame_id_ = frame_id;
    pub_ = nh.advertise<nav_msgs::Path>(topic_name, buffer_size);
}

void PathPub::Publish(const std::deque<Eigen::Matrix4f> &pose_queue, const double time_stamp)
{
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = frame_id_;

    for (size_t i = 0; i < pose_queue.size(); ++i)
    {
        Eigen::Matrix4f pose = pose_queue.at(i);

        geometry_msgs::PoseStamped pose_stamped;
        // ros::Time ros_time((float)time_stamp);
        // pose_stamped.header.stamp = ros_time;
        pose_stamped.header.frame_id = frame_id_;

        pose_stamped.header.seq = i;

        pose_stamped.pose.position.x = pose(0, 3);
        pose_stamped.pose.position.y = pose(1, 3);
        pose_stamped.pose.position.z = pose(2, 3);

        Eigen::Quaternionf q(pose.block<3, 3>(0, 0));

        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        path.poses.push_back(pose_stamped);
    }

    pub_.publish(path);
}

} // namespace Tools
