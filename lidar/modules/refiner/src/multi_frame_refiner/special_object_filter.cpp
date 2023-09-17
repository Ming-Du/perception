#include "multi_frame_refiner/special_object_filter.h"

namespace robosense {

SpecialObjectFilter::SpecialObjectFilter()
{

}

void SpecialObjectFilter::init(const RefinerParam& param){
    param_ = param;
}

void SpecialObjectFilter::perception(const LidarFrameMsg::Ptr &msg_ptr){

}

} // namespace robosense