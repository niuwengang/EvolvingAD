/**
 * @file    loc_node.cpp
 * @brief   veh for autopilot
 * @author  niu_wengang@163.com
 * @date    2024-04-28
 * @version v0.1.4
 */

// ros
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "loc_node");
    ros::NodeHandle loc_nh;

    std::cout << "loc node" << std::endl;
}

// class Scheduler
// {

//   public:
//     ros::NodeHandle nh;
// };

// {
//     ros::init(argc, argv, "loc_node");
//     Scheduler scheduler;

//     std::thread Thread1(&WorkFlow::func1Thread, &EX);
//     std::thread Thread2(&WorkFlow::func2Thread, &EX);
//     std::thread Thread3(&WorkFlow::func3Thread, &EX);

//     ros::Rate rate(20);
//     while (ros::ok())
//     {
//         ros::spinOnce();
//         rate.sleep();
//     }
//     Thread1.join();
//     Thread2.join();
//     Thread3.join();
//     return 0;
// }
