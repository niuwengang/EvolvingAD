#include "fix_gnss_sub.hpp"

FixGnssSub::FixGnssSub(ros::NodeHandle nh, const std::string topic_name, const size_t buffer_size)
{
    // 订阅 IMU 数据
    gnss_sub_ = nh.subscribe(topic_name, buffer_size, &FixGnssSub::MsgCallback, this);
}

void FixGnssSub::ParseData()
{
}

void FixGnssSub::MsgCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    ROS_INFO("GNSS Position: Latitude: %f, Longitude: %f, Altitude: %f", msg->latitude, msg->longitude, msg->altitude);
}
