#include "frame.hpp"

namespace evolving_ad_ns
{

Frame::Frame()
{
}

Frame &Frame::operator=(const Frame &other)
{
    if (this != &other)
    {
        this->time_stamp = other.time_stamp;   // timestamp
        this->pose = other.pose;               // pose
        this->cloud_msg = other.cloud_msg;     // cloud
        this->objects_msg = other.objects_msg; // objects
    }

    return *this;
}

Frame::Frame(const Frame &other)
{
    if (this != &other)
    {
        this->time_stamp = other.time_stamp;   // timestamp
        this->pose = other.pose;               // pose
        this->cloud_msg = other.cloud_msg;     // cloud
        this->objects_msg = other.objects_msg; // objects
    }
}

Frame::~Frame()
{
}

} // namespace evolving_ad_ns