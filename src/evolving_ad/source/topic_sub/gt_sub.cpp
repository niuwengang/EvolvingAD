#include "gt_sub.hpp"

namespace evolving_ad_ns
{

GtSub::GtSub(ros::NodeHandle &nh, const std::string topic_name, const size_t buffer_size)
{
    sub_ = nh.subscribe(topic_name, buffer_size, &GtSub::MsgCallback, this);
}
void GtSub::MsgCallback(const novatel_msgs::INSPVAXConstPtr &inspvax_ptr)
{
    mutex_.lock();

    GnssMsg gt_msg;

    const int LEAPSEC = 27;
    // double time_stamp = static_cast<double>(inspvax_ptr->header.gps_week * 7 * 24 * 3600 +
    //                                       inspvax_ptr->header.gps_week_seconds + (3652 * 86400) - LEAPSEC);

    gt_msg.time_stamp = ros::Time::now().toSec(); // time_stamp;
    gt_msg.latitude = inspvax_ptr->latitude;
    gt_msg.longitude = inspvax_ptr->longitude;
    gt_msg.altitude = inspvax_ptr->altitude;

    gt_msg.pitch = inspvax_ptr->pitch;
    gt_msg.roll = inspvax_ptr->pitch;
    gt_msg.azimuth = inspvax_ptr->azimuth;

    gt_msg_queue_.push_back(gt_msg);

    mutex_.unlock();
}

/**
 * @brief read gnss buffer
 * @param[in] gnss_msg_ptr
 * @return
 */
void GtSub::ParseData(std::deque<GnssMsg> &gt_msg_queue)
{
    if (gt_msg_queue_.size() > 0)
    {
        gt_msg_queue.insert(gt_msg_queue.end(), gt_msg_queue_.begin(), gt_msg_queue_.end());
        gt_msg_queue_.clear();
    }
}

} // namespace evolving_ad_ns