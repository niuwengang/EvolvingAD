/**
 * @file    imu_msg.hpp
 * @brief   imu消息封装
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 1.0
 * @note
 * https://docs.ros.org/en/jade/api/sensor_msgs/html/msg/NavSatStatus.html
 */
#ifndef GNSS_MSG_HPP
#define GNSS_MSG_HPP

// eigen lib
#include <Eigen/Core>
// geograph lib
#include <GeographicLib/LocalCartesian.hpp>
// c++ lib
#include <deque>
#include <iostream>

class GnssMsg
{

  public:
    enum class SatelliteFixStatus
    {
        STATUS_NO_FIX = -1, //  unable to fix position
        STATUS_FIX = 0,     // unaugmented fix
        STATUS_SBAS_FIX = 1,
        STATUS_GBAS_FIX = 2
    };

    enum class SatelliteNavService
    {
        SERVICE_GPS = 1,
        SERVICE_GLONASS = 2,
        SERVICE_COMPASS = 4, //     # includes BeiDou.
        SERVICE_GALILEO = 8
    };

  public:
    double time_stamp = 0.0;
    double longitude = 0.0; // 经度
    double latitude = 0.0;  // 纬度
    double altitude = 0.0;  // 高度

    double local_e = 0.0; // 东
    double local_n = 0.0; // 北
    double local_u = 0.0; // 天

    int status = -1;          // 状态 默认无定点
    unsigned int service = 1; // 卫星系统 默认GPS

  public:
    static bool TimeSync(std::deque<GnssMsg> &unsynced_gnss_msg_queue, std::deque<GnssMsg> &synced_gnss_msg_queue,
                         const double sync_time);

  private:
    static GeographicLib::LocalCartesian geo_converter_;
    static bool gnss_init_flag_; // 站心初始化

  public:
    void InitEnu();
    Eigen::Vector3f OdomUpdate();
};

#endif // GNSS_MSG_HPP