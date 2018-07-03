#ifndef __AEAG_HPP__
#define __AEAG_HPP__

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "sxc_common.hpp"

namespace sxc
{

class AEAG
{
public:
    AEAG();
    virtual ~AEAG();

    /***
     * /param mb Target mean brightness, 0 - 255. 
     */
    virtual void get_AEAG(cv::InputArray _m,
        xf exposure, xf gain, int mb, xf& newExposure, xf& newGain);

    xf get_exposure(void);
    xf get_gain(void);

    void set_exposure_top_limit(xf etl);
    xf   get_exposure_top_limit(void);

    void set_gain_top_limit(xf gtl);
    xf   get_gain_top_limit(void);

protected:
    xf mExposure;
    xf mGain;

    xf mExposureTopLimit;
    xf mGainTopLimit;
};


} // namespace sxc

#endif // __AEAG_HPP__