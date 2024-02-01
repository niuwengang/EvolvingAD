/**
 * @file    opencv_test.cpp
 * @brief   opencv_test 简单使用测试
 * @author  niu_wengang@163.com
 * @date    2024-01-21
 * @version 1.0
 */
#include <iostream>
#include <opencv2/opencv.hpp>
#include <unistd.h> //for get_current_dir_name

int main(int argc, char **argv)
{
    std::cout << "opencv 简单使用测试" << std::endl;

    const std::string project_folder_path = get_current_dir_name();
    const std::string image_file_path = project_folder_path + "/data/test/lane.jpg";

    cv::Mat image = cv::imread(image_file_path);
    if (image.empty())
    {
        std::cerr << "无法加载图片" << std::endl;
        return -1;
    }
    cv::imshow("lane", image);
    cv::waitKey(0);

    return 0;
}