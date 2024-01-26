#include "imu_sub.hpp"

ImuSub::ImuSub(ros::NodeHandle nh, const std::string topic_name, const size_t buffer_size)
{
    // 订阅 IMU 数据
    imu_sub_ = nh.subscribe(topic_name, buffer_size, &ImuSub::Callback, this);
    std::cout << "okok" << std::endl;
}

void ImuSub::ParseData()
{
}

void ImuSub::Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
}
