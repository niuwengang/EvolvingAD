#include "object_detection.hpp"

/**
 * @brief get device info
 * @param[in]
 * @return
 */
ObjectDetection::ObjectDetection(const std::string &model_file_path)
{
    model_file_path_ = model_file_path;

    /*2--get device info*/
    int count = 0;
    cudaGetDeviceCount(&count);
    spdlog::info("GPU num:{}", count);
    for (int index = 0; index < count; index++)
    {
        cudaDeviceProp prop;
        cudaGetDeviceProperties(&prop, index);
        spdlog::info("GPU name:{}", prop.name);
        spdlog::info("GPU memory:{}MB", prop.totalGlobalMem >> 20); // byte-->kb-->mb 2^(10+10)
        spdlog::info("GPU capbility:{} {}", prop.major, prop.minor);
    }

    checkCudaErrors(cudaStreamCreate(&stream_));

    pointpillar_ptr_ = std::make_shared<PointPillar>(model_file_path_, stream_); // pointpillar ptr init
    nms_pred_.reserve(100);
    setlocale(LC_ALL, "");
}

/**
 * @brief 3D objecttion detection
 * @param[in]
 * @return
 */
void ObjectDetection::Detect(const CloudMsg &cloud_msg, OdsMsg &ods_msg)
{
    nms_pred_.clear();         // clear
    ods_msg.ods_queue.clear(); // clear history
    const size_t num_points = cloud_msg.cloud_ptr->points.size();
    const size_t num_features = 4;
    float *d_points = new float[num_points * num_features];
    for (size_t i = 0; i < num_points; i++)
    {
        d_points[i * 4] = cloud_msg.cloud_ptr->points[i].x;
        d_points[i * 4 + 1] = cloud_msg.cloud_ptr->points[i].y;
        d_points[i * 4 + 2] = cloud_msg.cloud_ptr->points[i].z;
        d_points[i * 4 + 3] = 0.0f;
    }

    size_t points_size = cloud_msg.cloud_ptr->points.size();

    float *points_data = nullptr;                                                // destination data address
    unsigned int points_data_size = points_size * 4 * sizeof(float);             // destination data size
    checkCudaErrors(cudaMallocManaged((void **)&points_data, points_data_size)); // malloc memory for points_data
    checkCudaErrors(cudaMemcpy(points_data, d_points, points_data_size, cudaMemcpyDefault)); // copy a new one
    checkCudaErrors(cudaDeviceSynchronize());
    pointpillar_ptr_->doinfer(points_data, points_size, nms_pred_); // infer and out nms_pred_

    for (const auto box : nms_pred_)
    {
        OdMsg od_msg;

        od_msg.time_stamp = cloud_msg.time_stamp; // use cloud

        od_msg.x = box.x;
        od_msg.y = box.y;
        od_msg.z = box.z;

        od_msg.w = box.w;
        od_msg.l = box.l;
        od_msg.h = box.h;

        od_msg.heading = box.rt;
        od_msg.score = box.score;
        od_msg.id = box.id;

        if (od_msg.score > 0.6)
        {
            ods_msg.ods_queue.push_back(od_msg);
        }
        ods_msg.time_stamp = cloud_msg.time_stamp;
    }
}