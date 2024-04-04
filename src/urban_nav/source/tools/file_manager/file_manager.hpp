#ifndef FILE_MANAGER
#define FILE_MANAGER

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