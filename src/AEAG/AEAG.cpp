
#include <iostream>

#include "AEAG/AEAG.hpp"

using namespace sxc;

AEAG::AEAG()
: mExposure(0.0), mGain(0.0), mExposureTopLimit(1.0), mGainTopLimit(1.0)
{

}

AEAG::~AEAG()
{

}

void AEAG::get_AEAG(cv::InputArray _m, xf exposure, xf gain, int mb, xf& newExposure, xf& newGain, int* pLMB)
{
    
}

xf AEAG::get_exposure(void)
{
    return mExposure;
}

xf AEAG::get_gain(void)
{
    return mGain;
}

void AEAG::set_exposure_top_limit(xf etl)
{
    mExposureTopLimit = etl;
}

xf AEAG::get_exposure_top_limit(void)
{
    return mExposureTopLimit;
}

void AEAG::set_gain_top_limit(xf gtl)
{
    mGainTopLimit = gtl;
}

xf AEAG::get_gain_top_limit(void)
{
    return mGainTopLimit;
}
