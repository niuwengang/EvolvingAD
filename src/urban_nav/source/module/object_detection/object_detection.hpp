/**
 * @file    objection_detection.hpp
 * @brief   3D objection detection
 * @author  niu_wengang@163.com
 * @date    2024-04-05
 * @version 0.1.1
 */

#ifndef _OBJECT_DETECTION_HPP_
#define _OBJECT_DETECTION_HPP_

// cuda
#include <cuda_runtime.h>
// c++
#include <stdio.h>
// spdlog
#include <spdlog/spdlog.h>

#include "thirdpartylib/pointpillar/include/params.h"
#include "thirdpartylib/pointpillar/include/pointpillar.h"
class ObjectDetection
{
  public:
    ObjectDetection(const std::string &model_file_path);
    ~ObjectDetection() = default;
    // void GetResutl();

  private:
    void GetInfo();
    std::string model_file_path_ = "";
};

#endif // _OBJECT_DETECTION_HPP_
