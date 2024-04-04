/**
 * @file    odom_node.cpp
 * @brief   里程计启动节点
 * @author  niu_wengang@163.com
 * @date    2024-03-27
 * @version 1.0
 */

// related header
#include "pipe/odom_pipe.hpp"
// ros lib
#include <ros/ros.h>
// signal lib
#include <signal.h>

/**
 * @brief 信号中断函数
 * @param[in] sig
 * @return
 */
void sigintHandler(int sig)
{
    // 处理SIGINT信号（通常是用户按下Ctrl+C）
    std::cout << "odom_node节点关闭" << std::endl;
    ros::shutdown();
    exit(EXIT_SUCCESS);
}

/**
 * @brief main入口函数
 * @param[in] 略
 * @return
 */
int main(int argc, char **argv)
{
    std::cout << "odom_node节点启动" << std::endl;
    ros::init(argc, argv, "odom");
    ros::NodeHandle node_handle;
    signal(SIGINT, sigintHandler); // 注册SIGINT信号处理器

    std::shared_ptr<OdomPipe> odom_pipe_ptr = std::make_shared<OdomPipe>(node_handle); // odom pipe创建

    ros::Rate delay(10); // 1hz
    while (ros::ok())
    {
        ros::spinOnce();
        odom_pipe_ptr->Run();
        // delay.sleep(); // 频率控制
    }
    return 0;
}
