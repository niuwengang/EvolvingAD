/**
 * @file    front_end_node.cpp
 * @brief   front_end_node
 * @author  niu_wengang@163.com
 * @date    2024-04-05
 * @version v0.1.1
 */

#include "pipe/front_end_pipe.hpp"
// ros
#include <ros/ros.h>
// signal
#include <signal.h>
// spdlog
#include <spdlog/spdlog.h>
/**
 * @brief signal handle
 * @param[in] sig
 * @return
 */
void sigintHandler(int sig)
{
    spdlog::info("frontend_node$ shutdown");
    ros::shutdown();
    exit(EXIT_SUCCESS);
}

/**
 * @brief main function
 * @param[in] argc param nums
 * @param[in] argv param array
 * @return
 */
int main(int argc, char **argv)
{

    ros::init(argc, argv, "frontend_node");
    ros::NodeHandle nh;
    signal(SIGINT, sigintHandler);
    spdlog::info("frontend_node$ start");

    std::shared_ptr<FrontEndPipe> front_end_pipe_ptr = std::make_shared<FrontEndPipe>(nh);

    ros::Rate delay(100); // 100hz
    while (ros::ok())
    {
        ros::spinOnce();
        front_end_pipe_ptr->Run();
        delay.sleep();
    }
    return 0;
}
