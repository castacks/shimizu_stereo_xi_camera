#include <iostream>

#include "AEAG/MeanBrightness.hpp"

using namespace sxc;

MeanBrightness::MeanBrightness()
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

    gain = optimumEG / exposure;

    if ( gain > topG )
    {
        gain = topG;
    }
}

void MeanBrightness::get_AEAG(cv::InputArray _m, xf exposure, xf gain, int mb, xf& newExposure, xf& newGain)
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
    xf optEG = 1.0 * mb / mmb * exposure * gain;

    // Actual exposure-gain.
    split_exposure_gain(optEG, mPriority, mExposureTopLimit, mGainTopLimit, mExposure, mGain);

    // Transfer the values.
    newExposure = mExposure;
    newGain     = mGain;
}
