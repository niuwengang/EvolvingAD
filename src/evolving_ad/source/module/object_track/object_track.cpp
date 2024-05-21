#include "object_track.hpp"

namespace evolving_ad_ns
{

ObjectTrack::ObjectTrack()
{
}

void ObjectTrack::Track(const ObjectsMsg &objects_msg)
{ /*1.align cooridinates to now*/

    /*2.data association*/

    /*3.kalman filter predict*/

    /*4.birth and death memory*/

    
    if (first_flag_ == true)
    {

        first_flag_ = false;
    }
}
} // namespace evolving_ad_ns
