#include "gnss_odom.hpp"

namespace Module
{

GnssOdom::GnssOdom()
{
    gnss_odom_init_flag_ = false;
}

/**
 * @brief set gnss ENU origin pos
 * @param[in] gnss_msg
 * @return if set origin pos
 */
bool GnssOdom::InitPose(GnssMsg &gnss_msg)
{
    if (!gnss_odom_init_flag_)
    {
        gnss_msg.InitEnu();
        gnss_odom_init_flag_ = true;
    }
    return gnss_odom_init_flag_;
}

/**
 * @brief update gnss odom
 * @param[in out] gnss_odom
 * @param[in out] gnss_msg
 * @param[in out] imu_msg
 * @return
 */
bool GnssOdom::UpdateOdom(Eigen::Matrix4f &gnss_odom, GnssMsg &gnss_msg, ImuMsg &imu_msg)
{
    gnss_odom = Eigen::Matrix4f::Identity();

    gnss_odom.block<3, 1>(0, 3) = gnss_msg.OdomUpdate();
    gnss_odom.block<3, 3>(0, 0) = imu_msg.GetOrientationMatrix();

    return true;
}
} // namespace Module
