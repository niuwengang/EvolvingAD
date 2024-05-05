#ifndef _GT_SUB_HPP_
#define _GT_SUB_HPP_

// ros
#include <novatel_msgs/INSPVAX.h>
#include <ros/ros.h>
// msg
#include "msg/gnss_msg.hpp"

namespace evolving_ad_ns
{

class GtSub
{
  public:
    GtSub() = delete;
    GtSub(ros::NodeHandle &nh, const std::string topic_name, const size_t buffer_size = 10e5);
    ~GtSub() = default;
    void ParseData(std::deque<GnssMsg> &gt_msg_queue);

  private:
    void MsgCallback(const novatel_msgs::INSPVAXConstPtr &inspvax_ptr);

  private:
    std::deque<GnssMsg> gt_msg_queue_;
    ros::Subscriber sub_;
    std::mutex mutex_;
};

} // namespace evolving_ad_ns
#endif