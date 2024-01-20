/**
 * @file    yaml_test.cpp
 * @brief   yaml-cpp 简单使用测试
 * @author  niu_wengang@163.com
 * @date    2024-01-21
 * @version 1.0
 */

#include <fstream>
#include <iostream>
#include <unistd.h> //for get_current_dir_name
#include <yaml-cpp/yaml.h>

int main(int argc, char **argv)
{
    std::cout << "yaml-cpp 简单使用测试" << std::endl;

    const std::string project_folder_path = get_current_dir_name();
    const std::string yaml_file_path = project_folder_path + "/config/config.yaml";

    /*1--读取文件*/
    YAML::Node config = YAML::LoadFile(yaml_file_path);
    std::cout << "工程版本:" << config["project_info"]["version"] << std::endl;
    std::cout << "工程作者:" << config["project_info"]["author"] << std::endl;

    /*2--写入文件*/
    if (config["project_info"]["update"])
    {
        config["project_info"]["update"] = "2024-01-21";
    }

    std::ofstream file(yaml_file_path);
    file << config;
    file.close();

    return 0;
}