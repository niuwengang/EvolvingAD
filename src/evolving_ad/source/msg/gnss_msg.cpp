#include "gnss_msg.hpp"

namespace evolving_ad_ns
{
bool GnssMsg::gnss_init_flag_ = false;
GeographicLib::LocalCartesian GnssMsg::geo_converter_;

/**
 * @brief
 * @param[in]
 * @return
 */
void GnssMsg::InitEnu()
{
    geo_converter_.Reset(latitude, longitude, altitude);
    gnss_init_flag_ = true;
}

/**
 * @brief
 * @param[in]
 * @return
 */
Eigen::Vector3f GnssMsg::OdomUpdate()
{
    if (!gnss_init_flag_)
    {
        spdlog::error("gnss has not been inited");
        exit(EXIT_FAILURE);
    }

    geo_converter_.Forward(this->latitude, this->longitude, this->altitude, this->local_e, this->local_n,
                           this->local_u);
    return Eigen::Vector3f(this->local_e, this->local_n, this->local_u);
}

bool GnssMsg::TimeSync(std::deque<GnssMsg> &unsynced_gnss_msg_queue, GnssMsg &synced_gnss_msg, const double sync_time)
{
    while (unsynced_gnss_msg_queue.size() >= 2)
    {
        if (sync_time < unsynced_gnss_msg_queue.at(0).time_stamp)
        {
            return false;
        }
        if (unsynced_gnss_msg_queue.at(1).time_stamp < sync_time)
        {
            unsynced_gnss_msg_queue.pop_front();
            continue;
        }
        else
        {
            GnssMsg t0_gnss_msg = unsynced_gnss_msg_queue.at(0);
            GnssMsg t1_gnss_msg = unsynced_gnss_msg_queue.at(1);

            double coeff_t0 = (t1_gnss_msg.time_stamp - sync_time) / (t1_gnss_msg.time_stamp - t0_gnss_msg.time_stamp);
            double coeff_t1 = (sync_time - t0_gnss_msg.time_stamp) / (t1_gnss_msg.time_stamp - t0_gnss_msg.time_stamp);

            synced_gnss_msg.time_stamp = sync_time;

            synced_gnss_msg.longitude = t0_gnss_msg.longitude * coeff_t0 + t1_gnss_msg.longitude * coeff_t1;
            synced_gnss_msg.latitude = t0_gnss_msg.latitude * coeff_t0 + t1_gnss_msg.latitude * coeff_t1;
            synced_gnss_msg.altitude = t0_gnss_msg.altitude * coeff_t0 + t1_gnss_msg.altitude * coeff_t1;

            synced_gnss_msg.local_e = t0_gnss_msg.local_e * coeff_t0 + t1_gnss_msg.local_e * coeff_t1;
            synced_gnss_msg.local_n = t0_gnss_msg.local_n * coeff_t0 + t1_gnss_msg.local_n * coeff_t1;
            synced_gnss_msg.local_u = t0_gnss_msg.local_u * coeff_t0 + t1_gnss_msg.local_u * coeff_t1;

            return true;
        }
    }
    return false;
}

} // namespace evolving_ad_ns