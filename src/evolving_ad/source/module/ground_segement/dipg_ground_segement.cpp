#include "dipg_ground_segement.hpp"

namespace evolving_ad_ns
{

DipgGroundSegment::DipgGroundSegment()
{
    ground_segement_ptr_ = std::make_shared<DIPGSEG::Dipgseg>();
}

void DipgGroundSegment::Segement(const CloudMsg::CLOUD_PTR &cloud_in_ptr, CloudMsg::CLOUD_PTR &ground_cloud_ptr,
                                 CloudMsg::CLOUD_PTR &no_ground_cloud_ptr)
{
    ground_cloud_ptr.reset(new CloudMsg::CLOUD());
    no_ground_cloud_ptr.reset(new CloudMsg::CLOUD());

    ground_segement_ptr_->segment_ground(*cloud_in_ptr, *ground_cloud_ptr, *no_ground_cloud_ptr);
}
} // namespace evolving_ad_ns
