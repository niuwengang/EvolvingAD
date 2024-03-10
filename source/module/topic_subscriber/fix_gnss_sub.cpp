#include "fix_gnss_sub.hpp"



// 订阅 GNSS FIX 数据
FixGnssSub::FixGnssSub(ros::NodeHandle nh, const std::string topic_name, const size_t buffer_size)
{
    gnss_sub_ = nh.subscribe(topic_name, buffer_size, &FixGnssSub::MsgCallback, this);
}

void FixGnssSub::ParseData()
{
    
}

void FixGnssSub::MsgCallback(const sensor_msgs::NavSatFix::ConstPtr &msg_ptr)
{
    ROS_INFO("GNSS Position: Latitude: %f, Longitude: %f, Altitude: %f", msg_ptr->latitude, msg_ptr->longitude,
             msg_ptr->altitude);
}
