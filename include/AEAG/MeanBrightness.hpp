#ifndef __MEAN_BRIGHTNESS_HPP__
#define __MEAN_BRIGHTNESS_HPP__

#include "AEAG/ExposurePriorAEAG.hpp"

namespace sxc
{

class MeanBrightness : public ExposurePriorAEAG
{
public:
    MeanBrightness();
    virtual ~MeanBrightness();

    void get_AEAG(cv::InputArray _m, xf exposure, xf gain, int mb, xf& newExposure, xf& newGain);
};

} // namespace sxc

#endif // __MEAN_BRIGHTNESS_HPP__
