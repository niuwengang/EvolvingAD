

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;

    // 创建一个Publisher来发布消息
    ros::Publisher obj_pub = nh.advertise<jsk_recognition_msgs::BoundingBox>("bbx", 1000);

    // 循环发布消息
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        jsk_recognition_msgs::BoundingBox bbx;

        bbx.header.frame_id = "map";
        bbx.header.stamp = ros::Time::now();

        bbx.label = 0;             // 设置标签
        bbx.pose.position.x = 1.0; // 设置位置坐标
        bbx.pose.position.y = 2.0;
        bbx.pose.position.z = 3.0;
        bbx.dimensions.x = 0.5; // 设置尺寸
        bbx.dimensions.y = 0.3;
        bbx.dimensions.z = 0.2;
        obj_pub.publish(bbx);
        loop_rate.sleep();
    }

    return 0;
}
