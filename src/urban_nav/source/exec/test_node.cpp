/**
 * @file    test_node.cpp
 * @brief   test node
 * @author  niu_wengang@163.com
 * @date    2024-04-12
 * @version v0.1.1
 */

// ros
#include <ros/ros.h>
// signal
#include <signal.h>
// spdlog
#include <spdlog/spdlog.h>
// user msg
#include "user_msg/ods_msg.hpp"
// tools--pub
#include "tools/publisher/bbx_pub.hpp"

/**
 * @brief signal handle
 * @param[in] sig
 * @return
 */
void sigintHandler(int sig)
{
    spdlog::info("test_node$ shutdown");
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

    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    signal(SIGINT, sigintHandler);
    spdlog::info("test_node$ start");

    ros::Rate delay(100);

    OdsMsg ods_msg;
    OdMsg od_msg;
    od_msg.pos = Eigen::Vector3f(2, 5, 0);
    od_msg.dim = Eigen::Vector3f(3, 2, 2);
    ods_msg.ods_queue.push_back(od_msg);

    std::shared_ptr<Tools::BbxPub> bbx_pub_ptr = std::make_shared<Tools::BbxPub>(nh, "ods", "map");

    while (ros::ok())
    {
        ros::spinOnce();
        spdlog::info("this is test program");
        bbx_pub_ptr->Publish(ods_msg);
        delay.sleep();
    }
    return 0;
}
