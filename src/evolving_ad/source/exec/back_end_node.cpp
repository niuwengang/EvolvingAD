/**
 * @file    back_end_node.cpp
 * @brief   back end optimize node
 * @author  niu_wengang@163.com
 * @date    2024-04-10
 * @version v0.1.1
 */

#include "pipe/back_end_pipe.hpp"
// ros
#include <ros/ros.h>
// signal
#include <signal.h>
// spdlog
#include <spdlog/spdlog.h>

/**
 * @brief sign
 * @param[in] sig
 * @return
 */
void sigintHandler(int sig)
{

    spdlog::info("backend_node$ shutdown");
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

    ros::init(argc, argv, "backend_node");
    ros::NodeHandle nh;
    signal(SIGINT, sigintHandler);
    spdlog::info("backend_node$ start");

    std::shared_ptr<BackEndPipe> back_end_pipe_ptr = std::make_shared<BackEndPipe>(nh);

    ros::Rate delay(100); // 100hz
    while (ros::ok())
    {
        ros::spinOnce();
        back_end_pipe_ptr->Run();
        delay.sleep();
    }
    return 0;
}
