/**
 * @file    test_node.cpp
 * @brief   测试节点
 * @author  niu_wengang@163.com
 * @date    2024-03-27
 * @version 1.0
 */

// ros lib
#include <ros/ros.h>
// signal lib
#include <glog/logging.h> // glog 头文件
#include <signal.h>

void sigintHandler(int sig)
{
    // 处理SIGINT信号（通常是用户按下Ctrl+C）
    std::cout << "test_node节点关闭" << std::endl;
    ros::shutdown();
    exit(EXIT_SUCCESS);
}

/*main 入口函数*/
int main(int argc, char **argv)
{
    std::cout << "test_node节点启动" << std::endl;
    ros::init(argc, argv, "odom");
    ros::NodeHandle node_handle;
    signal(SIGINT, sigintHandler); // 注册SIGINT信号处理器

    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}
