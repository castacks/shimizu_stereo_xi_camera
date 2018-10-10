#include <iostream>
#include <cmath>

#include "AEAG/MeanBrightness.hpp"

using namespace sxc;

MeanBrightness::MeanBrightness()
: mCP(2000.0), mCD(500.0), mLastBDiff(0.0), mCT(10000)
{

}

MeanBrightness::~MeanBrightness()
{

}

static int get_mean_brightness(cv::InputArray _img)
{
    cv::Mat img = _img.getMat();

    int dataType = img.type();

    if ( dataType != CV_8U && dataType != CV_8UC1 )
    {
        std::cout << "get_mean_brightness(): dataType = " << dataType << std::endl;
        return -1;
    }

    int meanBrightness = (int)( cv::mean( img )[0] );

    return meanBrightness;
}

static void split_exposure_gain(xf optimumEG, xf expPriority, xf topE, xf topG, xf& exposure, xf& gain)
{
    exposure = expPriority * optimumEG;

    if ( exposure > topE )
    {
        exposure = topE;
    }

    if ( 1.0 == expPriority )
    {
        gain = 1.0;
    }
    else
    {
        gain = optimumEG / exposure;

        if ( gain > topG )
        {
            gain = topG;
        }
    }
}

void MeanBrightness::set_p(xf p)
{
    mCP = p;
}

void MeanBrightness::set_d(xf d)
{
    mCD = d;
}

xf MeanBrightness::get_p(void)
{
    return mCP;
}

xf MeanBrightness::get_d(void)
{
    return mCD;
}

void MeanBrightness::get_AEAG(cv::InputArray _m, xf exposure, xf gain, int mb, xf& newExposure, xf& newGain, int* pLMB)
{
    // Get the Mat object.
    cv::Mat m = _m.getMat();

    // Calculate the mean brightness of the Mat object.
    int mmb = get_mean_brightness(m);

    if ( -1 == mmb )
    {
        std::cout << "Error. Cannot calculate mean brightness." << std::endl;
        return;
    }

    // Optimum Exposure-gain.
    // xf optEG = 1.0 * mb / mmb * exposure * gain;
    xf currentBDiff = mb - mmb;
    xf optEG = exposure * gain + mCP * currentBDiff + mCD * ( currentBDiff - mLastBDiff );

    if ( optEG <= 0 )
    {
        std::cout << "Negative EG: " << optEG << std::endl;
        mExposure = exposure;
        mGain     = gain;
    }
    else if ( fabs(optEG - exposure * gain) > mCT )
    {
        // Actual exposure-gain.
        split_exposure_gain(optEG, mPriority, mExposureTopLimit, mGainTopLimit, mExposure, mGain);
    }
    else
    {
        mExposure = exposure;
        mGain     = gain;
    }
    
    // Transfer the values.
    newExposure = mExposure;
    newGain     = mGain;

    mLastBDiff = currentBDiff;

    if ( NULL != pLMB )
    {
        *pLMB = mmb;
    }
}

void MeanBrightness::put_image_parameters(cv::InputArray _m, int* pLMB)
{
    // Get the Mat object.
    cv::Mat m = _m.getMat();

    // Calculate the mean brightness of the Mat object.
    int mmb = get_mean_brightness(m);

    if ( -1 == mmb )
    {
        std::cout << "Error. Cannot calculate mean brightness." << std::endl;
        return;
    }

    if ( NULL != pLMB )
    {
        *pLMB = mmb;
    }
}
