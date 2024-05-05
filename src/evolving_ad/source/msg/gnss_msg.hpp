/**
 * @file    gnss_msg.hpp
 * @brief
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 1.0
 * @note
 * https://docs.ros.org/en/jade/api/sensor_msgs/html/msg/NavSatStatus.html
 */
#ifndef _GNSS_MSG_HPP_
#define _GNSS_MSG_HPP_

// spdlog
#include <spdlog/spdlog.h>
// eigen
#include <Eigen/Core>
// geograph
#include <GeographicLib/LocalCartesian.hpp>
// c++
#include <deque>
#include <iostream>

namespace evolving_ad_ns
{

class GnssMsg
{

  public:
    enum class SatelliteFixStatus
    {
        STATUS_NO_FIX = -1,  //  unable to fix position
        STATUS_FIX = 0,      // A few to tens of meters
        STATUS_SBAS_FIX = 1, // Satellite-Based Augmentation System,meters leve
        STATUS_GBAS_FIX = 2  // Ground-Based Augmentation System
    };

    enum class SatelliteNavService
    {
        SERVICE_GPS = 1,
        SERVICE_GLONASS = 2,
        SERVICE_COMPASS = 4, //     #  BeiDou.
        SERVICE_GALILEO = 8
    };

  public:
    double time_stamp = 0.0;
    double longitude = 0.0;
    double latitude = 0.0;
    double altitude = 0.0;

    double roll = 0.0;
    double pitch = 0.0;
    double azimuth = 0.0;

    double local_e = 0.0;
    double local_n = 0.0;
    double local_u = 0.0;

    int status = -1;
    unsigned int service = 1;

  public:
  private:
    static GeographicLib::LocalCartesian geo_converter_;
    static bool gnss_init_flag_;

  public:
    void InitEnu();
    Eigen::Vector3f OdomUpdate();
    static bool TimeSync(std::deque<GnssMsg> &unsynced_gnss_msg_queue, GnssMsg &synced_gnss_msg,
                         const double sync_time);
};

} // namespace evolving_ad_ns
#endif // GNSS_MSG_HPP