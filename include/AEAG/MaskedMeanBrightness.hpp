#ifndef __MASKED_MEAN_BRIGHTNESS_HPP__
#define __MASKED_MEAN_BRIGHTNESS_HPP__

#include <string>

#include "AEAG/MeanBrightness.hpp"

namespace sxc
{

class MaskedMeanBrightness : public MeanBrightness
{
public:
    MaskedMeanBrightness(const std::string& maskFn);
    virtual ~MaskedMeanBrightness();

    void read_exposure_mask(const std::string& fn);

private:
    void destroy(void);

protected:
    xf get_mean_brightness(cv::InputArray _img);

protected:
    int  mMaskN;
    int* mMaskX;
    int* mMaskY;
    xf*  mMaskF;
};

} // namespace sxc

#endif // __MASKED_MEAN_BRIGHTNESS_HPP__