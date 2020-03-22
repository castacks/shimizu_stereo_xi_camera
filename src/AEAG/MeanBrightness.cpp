#include <iostream>
#include <cmath>

#include "AEAG/MeanBrightness.hpp"

using namespace sxc;

MeanBrightness::MeanBrightness()
: ExposurePriorAEAG(), 
  mEP(1.0), mGP(1.0), mED(1.0), mGD(1.0), mEI(0.0), mGI(0.0), 
  mLastBDiff(0.0), mAccBDiff(0.0), 
  mCT(10000), mDEM(50000)
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

        // gain = gain * gain;

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

void MeanBrightness::set_i(xf ei, xf gi)
{
    mEI = ei;
    mGI = gi;
}

void MeanBrightness::set_dem(xf dem)
{
    mDEM = dem;
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

    // Optimum exposure-gain.
    // xf optEG = 1.0 * mb / mmb * exposure * gain;
    
    const xf alpha        = mmb / exposure; // A damping factor to smooth the control.
    const xf currentBDiff = mb - mmb;       // Current brightness difference.
    
    // Value for D-control.
    xf deltaE = mEP * currentBDiff + mED * (currentBDiff - mLastBDiff) + mEI * mAccBDiff;
    xf deltaG = mGP * currentBDiff + mGD * (currentBDiff - mLastBDiff) + mGI * mAccBDiff;

    // Damp the delta value.
    deltaE = deltaE / alpha;
    // deltaG = deltaG / alpha;

    // Clip the delta value.
    if ( deltaE > mDEM )
    {
        deltaE = mDEM;
    }
    else if ( deltaE < -mDEM )
    {
        deltaE = -mDEM;
    }

    // New exposure from the PD control.
    xf optExposure = exposure + mPriority * deltaE;
    newGain        = gain + ( 1.0f - mPriority ) * deltaG;
    
    // // Try to figure out the best combination of exposure and gain values.
    // split_exposure_gain( optExposure, mPriority, mExposureTopLimit, mGainTopLimit, newExposure, newGain );

    newExposure = optExposure;

    // Clip the exposure and gain values.
    if(newExposure < EXPOSURE_MIN)
    {
        newExposure = EXPOSURE_MIN; 
    }
    else if(newExposure > mExposureTopLimit)
    {
        newExposure = mExposureTopLimit;
    }

    newExposure = mExposureTopLimit;

    if (newGain < 1)
    {
        newGain = 1;
    }
    else if (newGain > mGainTopLimit)
    {
        newGain = mGainTopLimit;
    }
    
    // newGain = 5; // Debug use.

    // Transfer the values.
    mExposure = newExposure;
    mGain     = newGain;

    // // Debug.
    // std::cout.precision(3);
    // std::cout << std::scientific;
    // std::cout << "mb: " << mb << ", "
    //           << "mmb: " << mmb << ", "
    //           << "cBDiff: " << currentBDiff << ", "
    //           << "BDIff: " << mLastBDiff << ", "
    //           << "acc: " << mAccBDiff << ", "
    //           << "e: " << exposure << ", "
    //           << "g: " << gain << ", "
    //           << "optE: " << optExposure << ", "
    //           << "ne: " << mExposure << ", "
    //           << "ng: " << mGain << ", "
    //           << "a: " << alpha << ", "
    //           << "de: " << deltaE << ", "
    //           << "dg: " << deltaG << ", "
    //           << "topE: " << mExposureTopLimit << ", "
    //           << "topG: " << mGainTopLimit << std::endl;

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

    // Save the control value for next round of PD control.
    mLastBDiff = currentBDiff;
    mAccBDiff += currentBDiff;

    // Output value if required.
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
