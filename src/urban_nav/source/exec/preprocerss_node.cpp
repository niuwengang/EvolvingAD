/**
 * @file    preprocerss_node.cpp
 * @brief   preprocess node
 * @author  niu_wengang@163.com
 * @date    2024-04-09
 * @version v0.1.1
 */

#include "pipe/preprocerss_pipe.hpp"
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
    spdlog::info("preprocerss_node$ shutdown");
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

    ros::init(argc, argv, "preprocerss_node");
    ros::NodeHandle nh;
    signal(SIGINT, sigintHandler);
    spdlog::info("preprocerss_node$ start");

    std::shared_ptr<PreProcessPipe> preprocess_pipe_ptr = std::make_shared<PreProcessPipe>(nh);

    ros::Rate delay(100);

    while (ros::ok())
    {
        ros::spinOnce();
        preprocess_pipe_ptr->Run();
        delay.sleep();
    }
    return 0;
}
