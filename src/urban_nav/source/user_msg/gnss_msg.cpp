#include "gnss_msg.hpp"

// 静态成员变量必须在类外初始化
bool GnssMsg::pos_init_flag = false;
GeographicLib::LocalCartesian GnssMsg::geo_converter;

/**
 * @brief Gnss站心初始化
 * @param[in]
 * @return
 */
void GnssMsg::PosInit()
{
    geo_converter.Reset(latitude, longitude, altitude);
    pos_init_flag = true;
    // std::cout << "起点设置为:" << "经度:" << latitude << " 纬度:" << latitude << " 高度:" << altitude << std::endl;
}

/**
 * @brief WGS84转NEU
 * @param[in]
 * @return 东北天里程计
 */
Eigen::Vector3f GnssMsg::OdomUpdate()
{
    if (!pos_init_flag)
    {
        exit(EXIT_FAILURE);
        // std::cerr << "起点未设定" << std::endl;
    }

    double local_e, local_n, local_u;
    geo_converter.Forward(latitude, longitude, altitude, local_e, local_n, local_u);
    return Eigen::Vector3f(local_e, local_n, local_u);
}

bool GnssMsg::TimeSync(std::deque<GnssMsg> &unsynced_gnss_msg_queue, std::deque<GnssMsg> &synced_gnss_msg_queue,
                       const double sync_time)
{
    while (unsynced_gnss_msg_queue.size() >= 2)
    {
        if (sync_time < unsynced_gnss_msg_queue.at(0).time_stamp) // [0][1]超前于基准 退出等待
        {
            return false;
        }
        if (unsynced_gnss_msg_queue.at(1).time_stamp < sync_time) //[0][1]落后于基准 弹出轮循
        {
            unsynced_gnss_msg_queue.pop_front();
            continue;
        }
        else
        {
            GnssMsg t0_gnss_msg = unsynced_gnss_msg_queue.at(0);
            GnssMsg t1_gnss_msg = unsynced_gnss_msg_queue.at(1);
            GnssMsg synced_gnss_msg;

            double coeff_t0 = (t1_gnss_msg.time_stamp - sync_time) / (t1_gnss_msg.time_stamp - t0_gnss_msg.time_stamp);
            double coeff_t1 = (sync_time - t0_gnss_msg.time_stamp) / (t1_gnss_msg.time_stamp - t0_gnss_msg.time_stamp);

            /*时间戳*/
            synced_gnss_msg.time_stamp = sync_time;
            /*经维高*/
            synced_gnss_msg.longitude = t0_gnss_msg.longitude * coeff_t0 + t1_gnss_msg.longitude * coeff_t1;
            synced_gnss_msg.latitude = t0_gnss_msg.latitude * coeff_t0 + t1_gnss_msg.latitude * coeff_t1;
            synced_gnss_msg.altitude = t0_gnss_msg.altitude * coeff_t0 + t1_gnss_msg.altitude * coeff_t1;
            /*东北天*/
            synced_gnss_msg.local_e = t0_gnss_msg.local_e * coeff_t0 + t1_gnss_msg.local_e * coeff_t1;
            synced_gnss_msg.local_n = t0_gnss_msg.local_n * coeff_t0 + t1_gnss_msg.local_n * coeff_t1;
            synced_gnss_msg.local_u = t0_gnss_msg.local_u * coeff_t0 + t1_gnss_msg.local_u * coeff_t1;

            synced_gnss_msg_queue.push_back(synced_gnss_msg);

            return true;
        }
    }
    return false;
}