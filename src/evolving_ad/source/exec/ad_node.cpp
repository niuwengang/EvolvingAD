/**
 * @file    front_node.cpp
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
// msg

namespace evolving_ad_ns
{

class Scheduler
{

  public:
    void FrontEndThread()
    {
        ros::Rate rate_sub(10);
        std::shared_ptr<FrontEndPipe> front_end_ptr = std::make_shared<FrontEndPipe>(nh_, package_folder_path_);

        while (ros::ok())
        {
            front_end_ptr->Run();
            front_end_ptr->SendFrameQueue(frame_queue_);

            rate_sub.sleep();
        }
    }

    Scheduler(ros::NodeHandle &nh)
    {
        nh_ = nh;
        package_folder_path_ = ros::package::getPath("evolving_ad");
        std::thread Thread(&Scheduler::FrontEndThread, this);
        Thread.detach();
    }

  private:
    ros::NodeHandle nh_;
    std::string package_folder_path_;
    std::deque<Frame> frame_queue_;
};
} // namespace evolving_ad_ns
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ad_node");
    ros::NodeHandle nh("~");
    ros::Rate rate_main(100);

    std::shared_ptr<evolving_ad_ns::Scheduler> scheduler_ptr = std::make_shared<evolving_ad_ns::Scheduler>(nh);

    while (ros::ok())
    {

        ros::spinOnce();
        rate_main.sleep();
    }
    return 0;
}
