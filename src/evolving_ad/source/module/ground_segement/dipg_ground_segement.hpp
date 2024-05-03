/**
 * @file    ground_segement_interface.hpp
 * @brief   ground segement use Dipg method
 * @author  niu_wengang@163.com
 * @date    2024-04-23
 * @version 0.1.2
 */
#ifndef _DIPG_GROUND_SEGMENT_HPP_
#define _DIPG_GROUND_SEGMENT_HPP_

#include "ground_segement_interface.hpp"
#include "thirdpartylib/dipg_seg/include/dipgseg.h"

namespace evolving_ad_ns
{
class DipgGroundSegment : public GroundSegementInterface
{
  public:
    DipgGroundSegment();
    void Segement(const CloudMsg::CLOUD_PTR &cloud_in_ptr, CloudMsg::CLOUD_PTR &ground_cloud_ptr,
                  CloudMsg::CLOUD_PTR &no_ground_cloud_ptr) override;

  private:
    std::shared_ptr<DIPGSEG::Dipgseg> ground_segement_ptr_ = nullptr;
};
} // namespace evolving_ad_ns

#endif //_DIPG_GROUND_SEGMENT_HPP_