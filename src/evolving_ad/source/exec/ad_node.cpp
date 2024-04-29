/**
 * @file    loc_node.cpp
 * @brief   veh loc node for autopilot
 * @author  niu_wengang@163.com
 * @date    2024-04-28
 * @version v0.1.4
 */

// ros
#include <ros/package.h>
#include <ros/ros.h>
// thread
#include <thread>
//
#include "pipe/front_end.hpp"

class Scheduler
{

  public:
    void LocThread()
    {
        ros::Rate rate_sub(10);
        std::shared_ptr<evolving_ad_ns::FrontEndPipe> front_end_ptr =
            std::make_shared<evolving_ad_ns::FrontEndPipe>(nh_, package_folder_path_);

        while (ros::ok())
        {
            front_end_ptr->Run();
            rate_sub.sleep();
        }
    }

    Scheduler(ros::NodeHandle &nh)
    {
        nh_ = nh;
        package_folder_path_ = ros::package::getPath("evolving_ad");
        std::thread Thread1(&Scheduler::LocThread, this);
        Thread1.detach();
    }

  private:
    ros::NodeHandle nh_;
    std::string package_folder_path_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "loc_node");
    ros::NodeHandle nh("~");
    ros::Rate rate_main(100); // urbanloc: imu 100hz

    std::shared_ptr<Scheduler> scheduler_ptr = std::make_shared<Scheduler>(nh);

    while (ros::ok())
    {

        ros::spinOnce();
        rate_main.sleep();
    }
    return 0;
}
