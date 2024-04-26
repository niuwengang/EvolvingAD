/**
 * @file    file_manager.hpp
 * @brief   create folder stc
 * @author  niu_wengang@163.com
 * @date    2024-04-26
 * @version 0.1.3
 */

#ifndef _FILE_MANAGER_HPP_
#define _FILE_MANAGER_HPP_

// c++
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace Tools
{
class FileManager
{
  public:
    static bool CreateFolder(const std::string folder_path);
    static bool CreateTxtFile(std::ofstream &ofs, const std::string file_path);
};
} // namespace Tools

#endif