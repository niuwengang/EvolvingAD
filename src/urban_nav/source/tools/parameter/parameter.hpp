#include <Eigen/Core>
#include <string>

/**
 * @file    parameter.hpp
 * @brief   参数列表
 * @author  niu_wengang@163.com
 * @date    2024-03-29
 * @version 1.0
 * @note
 */

namespace Tools
{
class ParamList
{
  public:
    ParamList() = default;
    ~ParamList() = default;

    /*工程路径*/
    std::string package_folder_path;  // 功能包文件夹
    std::string result_folder_path;   // 结果存放文件夹
    std::string keyframe_folder_path; // 关键帧文件夹

    /*消息订阅*/
    std::string imu_sub_topic;          // imu订阅话题
    std::string cloud_sub_topic;        // 点云订阅话题
    std::string gnss_sub_topic;         // gnss订阅话题
    std::string ground_truth_sub_topic; // 真值订阅话题

    /*传感器使用*/
    bool use_imu = true;  // 是否使用imu
    bool use_gnss = true; // 是否使用gnss

    /*输出选项*/
    bool save_global_map = false;

    /*局部地图设置*/
    unsigned int keyframe_distance = 2.0;
    unsigned int keyframe_num = 20;

    /*空间外参*/
    Eigen::Matrix4f lidar_to_body = Eigen::Matrix4f::Identity(); // 雷达到车体系
    Eigen::Matrix4f imu_to_body = Eigen::Matrix4f::Identity();   // imu到车体系

    /*点云滤波*/
    float single_scan_filter_size = 1.0;
    float local_map_filter_size = 1.0;
    float global_map_filter_size = 1.0;

}; // namespace Toolsstruct ParamList
} // namespace Tools
