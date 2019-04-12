#include <iostream>
#include <cmath>

#include "AEAG/MeanBrightness.hpp"

using namespace sxc;

MeanBrightness::MeanBrightness()
: ExposurePriorAEAG(), 
  mEP(1.0), mGP(1.0), mED(1.0), mGD(1.0), mLastBDiff(0.0), mCT(10000)
{

}

MeanBrightness::~MeanBrightness()
{

}

xf MeanBrightness::get_mean_brightness(cv::InputArray _img)
{
    cv::Mat img = _img.getMat();

    int dataType = img.type();

    if ( dataType != CV_8U && dataType != CV_8UC1 )
    {
        std::cout << "get_mean_brightness(): dataType = " << dataType << std::endl;
        return -1;
    }

    xf meanBrightness = (xf)( cv::mean( img )[0] );

    return meanBrightness;
}

void MeanBrightness::split_exposure_gain(xf optimumEG, xf expPriority, xf topE, xf topG, xf& exposure, xf& gain)
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

void MeanBrightness::set_p(xf ep, xf gp)
{
    mEP = ep;
    mGP = gp;
}

void MeanBrightness::set_d(xf ed, xf gd)
{
    mED = ed;
    mGD = gd;
}

// xf MeanBrightness::get_p(void)
// {
//     return mCP;
// }

// xf MeanBrightness::get_d(void)
// {
//     return mCD;
// }

void MeanBrightness::get_AEAG(cv::InputArray _m, xf exposure, xf gain, int mb, xf& newExposure, xf& newGain, int* pLMB)
{
    // Get the Mat object.
    cv::Mat m = _m.getMat();

    // Calculate the mean brightness of the Mat object.
    xf mmb = this->get_mean_brightness(m);

    if ( mmb < 0 )
    {
        std::cout << "Error. Cannot calculate mean brightness." << std::endl;
        return;
    }

    // Optimum Exposure-gain.
    // xf optEG = 1.0 * mb / mmb * exposure * gain;
    xf currentBDiff = mb - mmb;
    
    xf deltaE = mEP * currentBDiff + mED * (currentBDiff - mLastBDiff);
    xf deltaG = mGP * currentBDiff + mGD * (currentBDiff - mLastBDiff);

    deltaE = mPriority * deltaE;
    deltaG = (1 - mPriority) * deltaG;

    if ( deltaE > 50000 )
    {
        deltaE = 50000;
    }
    else if ( deltaE < -50000 )
    {
        deltaE = -50000;
    }

    newExposure = exposure + deltaE;
    newGain = gain + deltaG;

    if(newExposure < EXPOSURE_MIN)
        newExposure = EXPOSURE_MIN; 
    else if(newExposure > mExposureTopLimit)
        newExposure = mExposureTopLimit;

    if (newGain < 1)
        newGain = 1;
    else if (newGain > mGainTopLimit)
        newGain = mGainTopLimit;

    mExposure = newExposure;
    mGain     = newGain;

    // Debug.
    std::cout << "mb: " << mb << ", "
              << "mmb: " << mmb << ", "
              << "currentBDiff: " << currentBDiff << ", "
              << "mLastBDIff: " << mLastBDiff << ", "
              << "exposure: " << exposure << ", "
              << "deltaE: " << deltaE << std::endl;

    // xf optEG = exposure * gain + mCP * currentBDiff + mCD * ( currentBDiff - mLastBDiff );

    // if ( optEG <= 0 )
    // {
    //     std::cout << "Negative EG: " << optEG << std::endl;
    //     mExposure = exposure;
    //     mGain     = gain;
    // }
    // else if ( fabs(optEG - exposure * gain) > mCT )
    // {
    //     // Actual exposure-gain.
    //     split_exposure_gain(optEG, mPriority, mExposureTopLimit, mGainTopLimit, mExposure, mGain);
    // }
    // else
    // {
    //     mExposure = exposure;
    //     mGain     = gain;
    // }
    
    // Transfer the values.
    // newExposure = mExposure;
    // newGain     = mGain;

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
    int mmb = this->get_mean_brightness(m);

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
