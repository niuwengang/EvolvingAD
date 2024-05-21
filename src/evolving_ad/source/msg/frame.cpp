#include "frame.hpp"

namespace evolving_ad_ns
{

Frame::Frame()
{
}

Frame &Frame::operator=(const Frame &other)
{
    this->time_stamp = other.time_stamp;                           // timestamp
    this->pose = other.pose;                                       // pose
    *(this->cloud_msg.cloud_ptr) = *(other.cloud_msg.cloud_ptr);   // cloud
    this->objects_msg.objects_vec = other.objects_msg.objects_vec; // ods_vec
    return *this;
}

Frame::Frame(const Frame &other)
{
    this->time_stamp = other.time_stamp;                           // timestamp
    this->pose = other.pose;                                       // pose
    *(this->cloud_msg.cloud_ptr) = *(other.cloud_msg.cloud_ptr);   // cloud
    this->objects_msg.objects_vec = other.objects_msg.objects_vec; // ods_vec
}

Frame::~Frame()
{
}

} // namespace evolving_ad_ns