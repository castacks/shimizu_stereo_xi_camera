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

    void set_p(xf ep, xf gp);
    void set_d(xf ed, xf gd);
    void set_t(int t);
    void set_dem(xf dem);

    // xf get_p(void);
    // xf get_d(void);

    void get_AEAG(cv::InputArray _m, xf exposure, xf gain, int mb, xf& newExposure, xf& newGain, int* pLMB = NULL);
    void put_image_parameters(cv::InputArray _m, int* pLMB);

protected:
    virtual xf get_mean_brightness(cv::InputArray _img);
    void split_exposure_gain(xf optimumEG, xf expPriority, xf topE, xf topG, xf& exposure, xf& gain);

protected:
    xf mEP, mGP; // The p coefficient of PD control.
    xf mED, mGD; // The d coefficient of PD control.
    xf mLastBDiff; // The brightness difference of the last round.
    int mCT; // The limit for the exposure adjustment, in microsecond.
    xf mDEM; // Delta exposure max.
};

} // namespace sxc

#endif // __MEAN_BRIGHTNESS_HPP__
