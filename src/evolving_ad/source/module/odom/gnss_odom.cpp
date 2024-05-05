#include "gnss_odom.hpp"

namespace evolving_ad_ns
{
GnssOdom::GnssOdom(const YAML::Node &config_node)
{
}
bool GnssOdom::InitPose(GnssMsg &gnss_msg)
{
    if (init_flag_ == false)
    {
        gnss_msg.InitEnu();
        init_flag_ = true;
    }
    return init_flag_;
}

void GnssOdom::ComputePose(GnssMsg &gnss_msg, Eigen::Matrix4f &new_pose)
{
    new_pose = Eigen::Matrix4f::Identity();

    new_pose.block<3, 1>(0, 3) = gnss_msg.OdomUpdate();
    // new_pose.block<3, 3>(0, 0) = imu_msg.GetOrientationMatrix();//reserve the orientation
}

} // namespace evolving_ad_ns