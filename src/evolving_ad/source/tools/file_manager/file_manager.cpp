/**
 * @file    file_manager.hpp
 * @brief   create folder stc
 * @author  niu_wengang@163.com
 * @date    2024-04-26
 * @version 0.1.3
 */

#include "file_manager.hpp"

namespace Tools
{

/**
 * @brief create a folder
 * @param[in] folder_path
 * @return bool
 * @note
 */
bool FileManager::CreateFolder(const std::string folder_path)
{
    std::filesystem::path folder_path_copy = folder_path;

    if (std::filesystem::exists(folder_path_copy))
    {
        return true;
    }
    else
    {
        std::filesystem::create_directory(folder_path_copy);
        if (std::filesystem::exists(folder_path_copy))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

/**
 * @brief create file
 * @param[in] ofs
 * @param[in] file_path
 * @return bool
 */
bool FileManager::CreateTxtFile(std::ofstream &ofs, const std::string file_path)
{
    // check if file exist
    if (std::filesystem::exists(file_path))
    {
        std::filesystem::remove(file_path);
    }

    ofs.open(file_path.c_str(), std::ios::app);
    if (ofs.is_open())
    {
        // ofs.close(); // 确保文件被关闭
        return true;
    }
    else
    {
        return false;
    }

    return true;
}

} // namespace Tools
