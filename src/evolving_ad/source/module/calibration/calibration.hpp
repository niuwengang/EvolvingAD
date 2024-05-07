#ifndef _CALIBRATION_HPP_
#define _CALIBRATION_HPP_

#include <Eigen/Core>
#include <pcl/registration/icp.h>

#include "msg/cloud_msg.hpp"

namespace evolving_ad_ns
{
class Lidar2GnssCalibration
{
  public:
    Lidar2GnssCalibration(const float distance_step, const unsigned int max_sampling_num);
    bool Calibration(const Eigen::Vector3f gnss_point, const Eigen::Vector3f lidar_point,
                     Eigen::Matrix4f &T_gnss2lidar);

  private:
    float distance_step_ = 0.1;
    float max_sampling_num_ = 20.0;

    std::vector<Eigen::Vector3f> gnss_point_vec_;
    std::vector<Eigen::Vector3f> lidar_point_vec_;

    double accumulate_gnss_odom_ = 0;
    double accumulate_lidar_odom_ = 0;
};
} // namespace evolving_ad_ns

#endif //_CALIBRATION_HPP_