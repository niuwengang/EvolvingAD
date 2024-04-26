#include "file_manager.hpp"

namespace Tools
{
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

bool FileManager::CreateTxtFile(std::ofstream &ofs, const std::string file_path)
{
    // 检查文件是否存在
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
