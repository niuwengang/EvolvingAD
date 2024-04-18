#include "object_detection.hpp"

ObjectDetection::ObjectDetection(const std::string &model_file_path)
{
    model_file_path_ = model_file_path;

    GetInfo();

    // get info
    // check cuda
}

/**
 * @brief
 * @param[in] none
 * @return
 */
void ObjectDetection::GetInfo()
{
    cudaDeviceProp prop; // check attributes
    int count = 0;
    cudaGetDeviceCount(&count);
    // spdlog::info("GPU has cuda devices:{}", count);
    //  std::cout << "cwd: " << getcwd(cwd, sizeof(cwd)) << std::endl;
    //  Model_File = std::string(cwd) + "/src/" + Model_File;

    //  printf("\nGPU has cuda devices: %d\n", count);
    //  for (int i = 0; i < count; ++i)
    //  {
    //      cudaGetDeviceProperties(&prop, i);
    //      printf("----device id: %d info----\n", i);
    //      printf("  GPU : %s \n", prop.name);
    //      printf("  Capbility: %d.%d\n", prop.major, prop.minor);
    //      printf("  Global memory: %luMB\n", prop.totalGlobalMem >> 20);
    //      printf("  Const memory: %luKB\n", prop.totalConstMem >> 10);
    //      printf("  SM in a block: %luKB\n", prop.sharedMemPerBlock >> 10);
    //      printf("  warp size: %d\n", prop.warpSize);
    //      printf("  threads in a block: %d\n", prop.maxThreadsPerBlock);
    //      printf("  block dim: (%d,%d,%d)\n", prop.maxThreadsDim[0], prop.maxThreadsDim[1], prop.maxThreadsDim[2]);
    //      printf("  grid dim: (%d,%d,%d)\n", prop.maxGridSize[0], prop.maxGridSize[1], prop.maxGridSize[2]);
    //  }
    //  printf("\n");
}
