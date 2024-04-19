/**
 * @file    objection_detection.hpp
 * @brief   3D objection detection
 * @author  niu_wengang@163.com
 * @date    2024-04-19
 * @version 0.1.1
 */

#ifndef _OBJECT_DETECTION_HPP_
#define _OBJECT_DETECTION_HPP_

// msg
#include "user_msg/cloud_msg.hpp"
#include "user_msg/ods_msg.hpp"
//  cuda
#include <cuda_runtime.h>
// c++
#include <stdio.h>
// spdlog
#include <spdlog/spdlog.h>
// tools-publisher
#include "tools/publisher/bbx_pub.hpp"
// thirdpartylib--pointpillar
#include "thirdpartylib/pointpillar/include/params.h"
#include "thirdpartylib/pointpillar/include/pointpillar.h"

#define checkCudaErrors(status)                                                                                        \
    {                                                                                                                  \
        if (status != 0)                                                                                               \
        {                                                                                                              \
            std::cout << "Cuda failure: " << cudaGetErrorString(status) << " at line " << __LINE__ << " in file "      \
                      << __FILE__ << " error status: " << status << std::endl;                                         \
            abort();                                                                                                   \
        }                                                                                                              \
    }

class ObjectDetection
{
  public:
    ObjectDetection(const std::string &model_file_path);
    ~ObjectDetection() = default;
    void Detect(const CloudMsg &cloud_msg, OdsMsg &ods_msg);

  private:
    std::string model_file_path_ = "";
    cudaStream_t stream_ = nullptr;

    std::shared_ptr<PointPillar> pointpillar_ptr_ = nullptr;
    std::vector<Bndbox> nms_pred_;
};

#endif // _OBJECT_DETECTION_HPP_
