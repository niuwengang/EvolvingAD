#include "../pipe/glins_pipe/glins_pipe.hpp"
#include <memory>
#include <ros/ros.h>
#include <signal.h>

void sigintHandler(int sig)
{
    // 处理SIGINT信号（通常是用户按下Ctrl+C）
    ROS_INFO("Received SIGINT, shutting down...");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;
    signal(SIGINT, sigintHandler); // 注册SIGINT信号处理器
    GlinsPipe glins_obj;

    // std::shared_ptr<GlinsPipe> glins_pipe_ptr = nullptr;
    // glins_pipe_ptr = std::make_shared<GlinsPipe>();
    ros::Rate delay(1); // 1hz

    return 0;
}