/**
 * @file    test_node.cpp
 * @brief   test node
 * @author  niu_wengang@163.com
 * @date    2024-04-12
 * @version v0.1.1
 */

// ros
#include <ros/package.h>
#include <ros/ros.h>
// signal
#include <signal.h>
// spdlog
#include <spdlog/spdlog.h>
// user msg
#include "user_msg/ods_msg.hpp"
// tools--pub
#include "tools/publisher/bbx_pub.hpp"
// module--object_detection
#include "module/object_detection/object_detection.hpp"

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

    std::string model_file_path = ros::package::getPath("urban_nav");

    std::shared_ptr<ObjectDetection> object_detection_ptr = std::make_shared<ObjectDetection>(model_file_path);

    while (ros::ok())
    {
        ros::spinOnce();
        spdlog::info("this is test program");

        delay.sleep();
    }
    return 0;
}

// OdsMsg ods_msg;
// OdMsg od_msg;
// od_msg.pos = Eigen::Vector3f(2, 5, 0);
// od_msg.dim = Eigen::Vector3f(3, 2, 2);
// ods_msg.ods_queue.push_back(od_msg);
// bbx_pub_ptr->Publish(ods_msg);
// std::shared_ptr<Tools::BbxPub> bbx_pub_ptr = std::make_shared<Tools::BbxPub>(nh, "ods", "map");