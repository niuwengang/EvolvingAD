#include "module/topic_subscriber/fix_gnss_sub.hpp"
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h> // 包含GNSS消息类型

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gnss_listener"); // 初始化节点
    ros::NodeHandle nh;                     // 创建节点句柄
    FixGnssSub fix_gnss_sub_ptr(nh, "/ublox_driver/receiver_lla", 10);
    ros::spin(); // 循环等待消息

    return 0;
}
