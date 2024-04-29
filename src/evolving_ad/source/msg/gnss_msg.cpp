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

    double local_e, local_n, local_u;
    geo_converter_.Forward(latitude, longitude, altitude, local_e, local_n, local_u);
    return Eigen::Vector3f(local_e, local_n, local_u);
}

}