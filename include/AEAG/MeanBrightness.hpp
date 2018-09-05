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

    void set_p(xf p);
    void set_d(xf d);
    void set_t(int t);

    xf get_p(void);
    xf get_d(void);

    void get_AEAG(cv::InputArray _m, xf exposure, xf gain, int mb, xf& newExposure, xf& newGain);

protected:
    xf mCP; // The p coefficient of PD control.
    xf mCD; // The d coefficient of PD control.
    xf mLastBDiff; // The brightness difference of the last round.
    int mCT; // The limit for the exposure adjustment, in microsecond.
};

} // namespace sxc

#endif // __MEAN_BRIGHTNESS_HPP__
