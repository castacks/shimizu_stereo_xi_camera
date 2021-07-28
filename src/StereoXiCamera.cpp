
#include <pthread.h>

#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>

#include "Profiler/Profiler.hpp"

#include "sxc_common.hpp"
#include "StereoXiCamera.hpp"

using namespace sxc;

StereoXiCamera::StereoXiCamera(std::string &camSN0, std::string &camSN1)
: AUTO_GAIN_EXPOSURE_PRIORITY_MAX(1.0), AUTO_GAIN_EXPOSURE_PRIORITY_MIM(0.49), AUTO_GAIN_EXPOSURE_PRIORITY_DEFAULT(0.8),
  AUTO_GAIN_EXPOSURE_TARGET_LEVEL_MAX(60.0), AUTO_GAIN_EXPOSURE_TARGET_LEVEL_MIN(10.0), AUTO_GAIN_EXPOSURE_TARGET_LEVEL_DEFAULT(40.0),
  AUTO_EXPOSURE_TOP_LIMIT_MAX(900000), AUTO_EXPOSURE_TOP_LIMIT_MIN(1), AUTO_EXPOSURE_TOP_LIMIT_DEFAULT(200000),
  AUTO_GAIN_TOP_LIMIT_MAX(36.0), AUTO_GAIN_TOP_LIMIT_MIN(0.0), AUTO_GAIN_TOP_LIMIT_DEFAULT(12.0),
  TOTAL_BANDWIDTH_MAX(4000), TOTAL_BANDWIDTH_MIN(2400),
  BANDWIDTH_MARGIN_MAX(50), BANDWIDTH_MARGIN_MIN(0), BANDWIDTH_MARGIN_DEFAULT(10),
  TRIGGER_SOFTWARE(1), EXPOSURE_MILLISEC_BASE(1000), CAM_IDX_0(0), CAM_IDX_1(1),
  XI_DEFAULT_TOTAL_BANDWIDTH(2400), XI_DEFAULT_BANDWIDTH_MARGIN(10),
  mHeight(3008), mWidth(4112),
  mXi_DownsamplingType(XI_SKIPPING), mXi_Downsampling(XI_DWN_1x1), 
  mFixedXiExposureGain(false),
  mXi_AutoGainExposurePriority(AUTO_GAIN_EXPOSURE_PRIORITY_DEFAULT),
  mXi_AutoExposureTopLimit(AUTO_EXPOSURE_TOP_LIMIT_DEFAULT),
  mXi_AutoGainTopLimit(AUTO_GAIN_TOP_LIMIT_DEFAULT),
  mXi_BandwidthMargin(BANDWIDTH_MARGIN_DEFAULT),
  mTransferFormat(TF_COLOR),
  mIsExternalTriggered(false), mXi_NextImageTimeout_ms(1000),
  mIsExternalTimeStampReset(false), 
  mSelfAdjustNumOmittedFrames(5), mSelfAdjustNumFrames(5), 
  mSelfAdjustNumTrialLoops((mSelfAdjustNumOmittedFrames+mSelfAdjustNumFrames)*2), mIsSelfAdjusting(false),
  mXi_Exposure(100), mXi_Gain(0),  
  mCAEAG(NULL), mCAEAG_TargetBrightnessLevel(10), mCAEAG_TargetBrightnessLevel8Bit(0), mCAEAG_IsEnabled(false),
  mRawBalance(false),
  mForceXiAutoWhiteBalance(false),
  mFixedWB(true), mWB_R(1.3), mWB_G(1.0), mWB_B(2.7),
  mIPE(NULL),
  mFlagDebug(false)
{
    mCamSN[CAM_IDX_0] = camSN0;
    mCamSN[CAM_IDX_1] = camSN1;

    mCAEAG_TargetBrightnessLevel8Bit = (int)(mCAEAG_TargetBrightnessLevel / 100.0 * 255);

    if ( mCAEAG_TargetBrightnessLevel8Bit > 255 )
    {
        mCAEAG_TargetBrightnessLevel8Bit = 255;
    }
}

StereoXiCamera::~StereoXiCamera()
{

}

void StereoXiCamera::open()
{
    try
    {
        prepare_before_opening();
        open_and_common_settings();
    }
    catch ( xiAPIplus_Exception& exp )
    {
        EXCEPTION_CAMERA_API(exp);
    }
}

void StereoXiCamera::self_adjust(bool verbose)
{
    // Check if the white balance is fixed.
    if ( true == mFixedWB )
    {
        BOOST_THROW_EXCEPTION( exception_base() << ExceptionInfoString( "Cannot perform self adjust with fixed white balance." ) );
    }

    mIsSelfAdjusting = true;

    // Take several images and record the settings.
    std::vector<CameraParams_t> camParams;

    if ( true == verbose )
    {
        std::cout << "Begin self-adjust..." << std::endl;
    }

    LOOP_CAMERAS_BEGIN
        mCams[loopIdx].EnableWhiteBalanceAuto();
    LOOP_CAMERAS_END

    if ( TF_RAW == mTransferFormat )
    {
        LOOP_CAMERAS_BEGIN
            set_transfer_format_single_camera( mCams[loopIdx], TF_COLOR );
        LOOP_CAMERAS_END
    }

    record_settings(mSelfAdjustNumFrames, camParams, verbose);

    if ( TF_RAW == mTransferFormat )
    {
        LOOP_CAMERAS_BEGIN
            set_transfer_format_single_camera( mCams[loopIdx], TF_RAW );
        LOOP_CAMERAS_END
    }

    LOOP_CAMERAS_BEGIN
        mCams[loopIdx].DisableWhiteBalanceAuto();
    LOOP_CAMERAS_END

    if ( true == verbose )
    {
        std::cout << "Adjust exposure and gain." << std::endl;
    }

    self_adjust_exposure_gain(camParams);

    if ( true == verbose )
    {
        std::cout << "Adjust white balance." << std::endl;
    }

    self_adjust_white_balance(camParams);
    
    if ( true == verbose )
    {
        std::cout << "Self-adjust done." << std::endl;
    }

    mIsSelfAdjusting = false;
}

void StereoXiCamera::record_settings(int nFrames, std::vector<CameraParams_t> &vcp, bool verbose)
{
    // Temporary variables.
    cv::Mat cvImages[2];                  // OpenCV Mat array to hold the images.
    StereoXiCamera::CameraParams_t cp[2]; // Camera parameters.

    // Set the trigger source.
    set_stereo_software_trigger();

    // Start acquisition.
    start_acquisition();

    int i = 0;
    int j = 0;
    int resGetImages = -1;

    while ( i < mSelfAdjustNumOmittedFrames + mSelfAdjustNumFrames && j < mSelfAdjustNumTrialLoops )
    {
        // Software trigger.
        software_trigger(true);
        
        // Get images.
        resGetImages = get_images( cvImages[0], cvImages[1], cp[0], cp[1] );

        if ( 0 == resGetImages )
        {
            // get_images() succeeds.
            if ( true == verbose )
            {
                std::cout << "Self-adjust image No. " << i + 1 
                        << " with " << mSelfAdjustNumOmittedFrames 
                        << " to omit." << std::endl; 
            }

            if ( i >= mSelfAdjustNumOmittedFrames )
            {
                // Record the parameters.
                vcp.push_back( cp[0] );
                vcp.push_back( cp[1] );
            }

            ++i;
        }

        ++j;
    }

    // Stop acquisition.
    stop_acquisition();

    // Restore the trigger settings.
    if ( false == mIsExternalTriggered )
    {
        set_stereo_master_trigger();
    }
    else
    {
        set_stereo_external_trigger();
    }

    // Check the status.
    if ( j == mSelfAdjustNumTrialLoops )
    {
        if ( i == mSelfAdjustNumOmittedFrames + mSelfAdjustNumFrames )
        {
            // We are fine.
        }
        else if ( i < mSelfAdjustNumOmittedFrames + mSelfAdjustNumFrames )
        {
            // This is an error.
            EXCEPTION_BAD_HARDWARE_RESPONSE("Record settings for self-adjust failed with bad hardware response.");
        }
    }
}

void StereoXiCamera::self_adjust_exposure_gain(std::vector<CameraParams_t> &cp)
{
    // Calculate the averaged exposure and gain settings.
    int n = cp.size();

    int avgExposure = 0;
    xf  avgGain     = 0.0;
    
    std::vector<CameraParams_t>::iterator iter;

    for ( iter = cp.begin(); iter != cp.end(); iter++ )
    {
        avgExposure += (*iter).exposure;
        avgGain     += (*iter).gain;
    }

    avgExposure = (int)( 1.0 * avgExposure / n );
    avgGain     = avgGain / n;

    // Apply the exposure and gain settings to the cameras.
    LOOP_CAMERAS_BEGIN
        set_exposure_gain(loopIdx, avgExposure, avgGain);
    LOOP_CAMERAS_END

    mXi_Exposure = avgExposure;
    mXi_Gain     = avgGain;
}

void StereoXiCamera::enable_downsampling(void)
{
    mXi_DownsamplingType = XI_SKIPPING;
    mXi_Downsampling     = XI_DWN_2x2;
}

void StereoXiCamera::disable_downsampling(void)
{
    mXi_DownsamplingType = XI_SKIPPING;
    mXi_Downsampling     = XI_DWN_1x1;
}

void StereoXiCamera::enable_fixed_exposure_gain(void)
{
    // TODO: Error checking with customized AEAG.
    mFixedXiExposureGain = true;
}

void StereoXiCamera::disable_fixed_exposure_gain(void)
{
    mFixedXiExposureGain = false;
}

void StereoXiCamera::set_exposure_gain(int idx, int e, xf g)
{
    // Disable the auto exposure auto gain (AEAG).
    mCams[idx].DisableAutoExposureAutoGain();

    // Set the parameters.
    mCams[idx].SetExposureTime( e );
    mCams[idx].SetGain( g );
}

void StereoXiCamera::set_white_balance(int idx, xf r, xf g, xf b)
{
    // Disable the auto white balance.
    mCams[idx].DisableWhiteBalanceAuto();

    // Set the parameters.
    mCams[idx].SetWhiteBalanceRed(r);
    mCams[idx].SetWhiteBalanceGreen(g);
    mCams[idx].SetWhiteBalanceBlue(b);
}

void StereoXiCamera::set_white_balance(xiAPIplusCameraOcv& cam, xf r, xf g, xf b)
{
    // Disable the auto white balance.
    cam.DisableWhiteBalanceAuto();

    // Set the parameters.
    cam.SetWhiteBalanceRed(r);
    cam.SetWhiteBalanceGreen(g);
    cam.SetWhiteBalanceBlue(b);
}

void StereoXiCamera::self_adjust_white_balance(std::vector<CameraParams_t> &cp)
{
    // Calculate the averaged white balance settings.
    int n = cp.size();

    xf avgR = 0.0;
    xf avgG = 0.0;
    xf avgB = 0.0;

    std::vector<CameraParams_t>::iterator iter;

    for ( iter = cp.begin(); iter != cp.end(); ++iter )
    {
        avgR += (*iter).AWB_kr;
        avgG += (*iter).AWB_kg;
        avgB += (*iter).AWB_kb;
    }

    avgR /= n;
    avgG /= n;
    avgB /= n;

    // Apply the white balance settings to the cameras.
    LOOP_CAMERAS_BEGIN
        set_white_balance(loopIdx, avgR, avgG, avgB);
    LOOP_CAMERAS_END

    mWB_R = avgR;
    mWB_G = avgG;
    mWB_B = avgB;
}

void StereoXiCamera::apply_custom_AEAG(cv::Mat &img0, cv::Mat &img1, CameraParams_t &camP0, CameraParams_t &camP1)
{
    if ( NULL == mCAEAG )
    {
        EXCEPTION_ARG_NULL(mCAEAG);
    }

    int currentExposureUS[2] = { camP0.exposure, camP1.exposure };
    // xf  currentGainDB[2]     = { camP0.gain, camP1.gain };
    xf  currentGainDB[2]     = { mXi_Gain, mXi_Gain };

    xf currentExposure[2] = {0.0, 0.0};
    xf currentGain[2]     = {0.0, 0.0};
    xf newExposure[2]     = {0.0, 0.0};
    xf newGain[2]         = {0.0, 0.0};

    LOOP_CAMERAS_BEGIN
        currentExposure[loopIdx] = currentExposureUS[loopIdx];
        currentGain[loopIdx]     = dBToGain(currentGainDB[loopIdx]);
    LOOP_CAMERAS_END

    // Convert images.
    switch(mTransferFormat)
    {
        case TF_COLOR:
        {
            cv::cvtColor( img0, mGrayMatBuffer[CAM_IDX_0], cv::COLOR_BGR2GRAY, 1 );
            cv::cvtColor( img1, mGrayMatBuffer[CAM_IDX_1], cv::COLOR_BGR2GRAY, 1 );
            break;
        }
        case TF_MONO:
        {
            mGrayMatBuffer[CAM_IDX_0] = img0;
            mGrayMatBuffer[CAM_IDX_1] = img1;
            break;
        }
        case TF_RAW:
        {
            mGrayMatBuffer[CAM_IDX_0] = img0;
            mGrayMatBuffer[CAM_IDX_1] = img1;
            break;
        }
        default:
        {
            BOOST_THROW_EXCEPTION( exception_base() << ExceptionInfoString("Cannot perform custom AEAG with undefined transfer format.") );
        }
    }

    // The first camera.
    mCAEAG->get_AEAG(mGrayMatBuffer[CAM_IDX_0], 
        currentExposure[CAM_IDX_0], currentGain[CAM_IDX_0], 
        mCAEAG_TargetBrightnessLevel8Bit, 
        newExposure[CAM_IDX_0], newGain[CAM_IDX_0],
        mMeanBrightness + CAM_IDX_0);

    // // The second camera.
    // mCAEAG->get_AEAG(mGrayMatBuffer[CAM_IDX_1], 
    //     currentExposure[CAM_IDX_1], currentGain[CAM_IDX_1], 
    //     mCAEAG_TargetBrightnessLevel8Bit, 
    //     newExposure[CAM_IDX_1], newGain[CAM_IDX_1],
    //     mMeanBrightness + CAM_IDX_1);

    // // Average.
    // int avgExposure = (int)(0.5 * ( newExposure[0] + newExposure[1] ));
    // xf  avgGain     = 0.5 * ( newGain[0] + newGain[1] );

    int avgExposure = (int)( newExposure[0] );
    xf  avgGain     = newGain[0];

    if ( true == mFlagDebug )
    {
        std::cout << "avgGain = " << avgGain << std::endl;
    }

    if ( true == std::isnan(avgGain) )
    {
        std::cout << "avgGain is nan" << std::endl;
    }

    avgGain = GainToDB(avgGain);
    mXi_Gain = avgGain;

    // std::cout << "avgGain = " << avgGain << std::endl;

    // Apply exposure and gain settings.
    LOOP_CAMERAS_BEGIN
        // For test use.
        if ( true == mFlagDebug )
        // if ( true )
        {
            std::cout << "Cam " << loopIdx 
                  << ", avgExposure (ms) = " << avgExposure / 1000.0
                  << ", avgGain (dB) = " << avgGain
                  << std::endl;
        }

        try
        {
            mCams[loopIdx].SetExposureTime(avgExposure);
            mCams[loopIdx].SetGain(avgGain);
        }
        catch ( xiAPIplus_Exception& exp )
        {
            std::cout << "*** AEAG *** " 
                      << "avgExcposure = " << avgExposure << ", "
                      << "avgGain = " << avgGain << ". " << std::endl;
            throw exp;
        }
    LOOP_CAMERAS_END
}

void StereoXiCamera::evaluate_image_parameters(
    cv::Mat &img0, cv::Mat &img1, CameraParams_t &camP0, CameraParams_t &camP1)
{
    // The first camera.
    switch(mTransferFormat)
    {
        case TF_COLOR:
        {
            cv::cvtColor( img0, mGrayMatBuffer[CAM_IDX_0], cv::COLOR_BGR2GRAY, 1 );
            cv::cvtColor( img1, mGrayMatBuffer[CAM_IDX_1], cv::COLOR_BGR2GRAY, 1 );
            break;
        }
        case TF_MONO:
        {
            mGrayMatBuffer[CAM_IDX_0] = img0;
            mGrayMatBuffer[CAM_IDX_1] = img1;
            break;
        }
        case TF_RAW:
        {
            mGrayMatBuffer[CAM_IDX_0] = img0;
            mGrayMatBuffer[CAM_IDX_1] = img1;
            break;
        }
        default:
        {
            BOOST_THROW_EXCEPTION( exception_base() << ExceptionInfoString("Cannot evaluate image parameters with undefined transfer format.") );
        }
    }
    
    mIPE->put_image_parameters( mGrayMatBuffer[CAM_IDX_0], mMeanBrightness + CAM_IDX_0 );
    mIPE->put_image_parameters( mGrayMatBuffer[CAM_IDX_1], mMeanBrightness + CAM_IDX_1 );
}

void StereoXiCamera::start_acquisition(int waitMS)
{
    try
    {
        if ( true == mCAEAG_IsEnabled && false == mIsSelfAdjusting )
        {
            LOOP_CAMERAS_BEGIN
                mCams[loopIdx].DisableAutoExposureAutoGain();
            LOOP_CAMERAS_END
        }

        // Debug use.
        if ( true == mForceXiAutoWhiteBalance )
        {
            LOOP_CAMERAS_BEGIN
                mCams[loopIdx].EnableWhiteBalanceAuto();
            LOOP_CAMERAS_END
        }

        LOOP_CAMERAS_BEGIN
            mCams[loopIdx].StartAcquisition();
        LOOP_CAMERAS_END
    }
    catch ( xiAPIplus_Exception& exp )
    {
        EXCEPTION_CAMERA_API(exp);
    }

    // Wait for a short period of time.
    // cvWaitKey(waitMS); // OpenCV 4.5 has trouble with this.
    std::this_thread::sleep_for(std::chrono::milliseconds(waitMS));
}

void StereoXiCamera::software_trigger(bool both)
{
    if ( true == mIsExternalTriggered && 
         false == mIsSelfAdjusting )
    {
        std::cout << "External trigger enabled. Software trigger is ignored." << std::endl;
        return;
    }

    try
    {
        // Trigger.
        mCams[CAM_IDX_0].SetTriggerSoftware(TRIGGER_SOFTWARE);
        if ( true == both )
        {
            mCams[CAM_IDX_1].SetTriggerSoftware(TRIGGER_SOFTWARE);
        }
    }
    catch ( xiAPIplus_Exception& exp )
    {
        EXCEPTION_CAMERA_API(exp);
    }
}

cv::Mat StereoXiCamera::get_single_image(int idx)
{
    // Obtain the images.
    XI_IMG_FORMAT format;
    cv::Mat cv_mat_image;

    format = mCams[idx].GetImageDataFormat();
    // std::cout << "format = " << format << std::endl;
    
    cv_mat_image = mCams[idx].GetNextImageOcvMat();
    
    if (format == XI_RAW16 || format == XI_MONO16)
    {
        normalize(cv_mat_image, cv_mat_image, 0, 65536, cv::NORM_MINMAX, -1, cv::Mat()); // 0 - 65536, 16 bit unsigned integer range
    }

    return cv_mat_image;
}

// Data structures for the multi-threaded version of get_single_iamge.
typedef struct TArg_get_single_image
{
    std::string name;
    xiAPIplusCameraOcv* cam;
    int flag;
    cv::Mat image;
    cv::Mat* grayBuffer;
    DWORD ts[2];
    int n; // Number of images to get.
    int c; // Number of images already got.
} TArg_get_single_image_t;

/** A multi-thread version of get_single_image
 * 
 * This function will catch the timeout exception from Ximea API.
 * If timeout, flag = 2. If other exception, flag = 3.
 * 
 * If an image is obtained successfully, flag will be 1. The obtained 
 * image could be found in image member variable.
 * 
 */
static void *
thd_get_single_image(void* arg)
{
    // Cast the input argument.
    TArg_get_single_image_t* a = (TArg_get_single_image_t*)arg;
    a->c = 0;
    
    // Check if we need to get any image.
    if ( 0 == a->n )
    {
        a->flag = 1;
        return (void*)( &(a->flag) );
    }

    a->ts[0] = 0;
    a->ts[1] = 0;

    // Obtain the images.
    XI_IMG_FORMAT format;
    // XI_COLOR_FILTER_ARRAY filter;
    cv::Mat cv_mat_image;

    // For profiler.
    std::string profilerName_GetImageDataFormat = "GetImageDataFormat_" + a->name;
    std::string profilerName_GetNextImageOcvMat = "GetNextImageOcvMat_" + a->name;

    try
    {
        PROFILER_IN(profilerName_GetImageDataFormat.c_str());
        format = a->cam->GetImageDataFormat();
        PROFILER_OUT(profilerName_GetImageDataFormat.c_str());
        // std::cout << "format = " << format << std::endl;
        // filter = a->cam->GetSensorColorFilterArray();
        // std::cout << "filter = " << filter << std::endl;
        
        PROFILER_IN(profilerName_GetNextImageOcvMat.c_str());
        for ( int i = 0; i < a->n; i++ )
        {
            cv_mat_image = a->cam->GetNextImageOcvMat( a->ts );
            a->c++; // Count the number of images.
        }
        PROFILER_OUT(profilerName_GetNextImageOcvMat.c_str());
        
        if (format == XI_RAW16 || format == XI_MONO16)
        {
            normalize(cv_mat_image, cv_mat_image, 0, 65536, cv::NORM_MINMAX, -1, cv::Mat()); // 0 - 65536, 16 bit unsigned integer range
        }

        a->image = cv_mat_image;

        if ( a->grayBuffer->rows != a->image.rows ||
             a->grayBuffer->cols != a->image.cols )
        {
            *(a->grayBuffer) = cv::Mat::zeros(a->image.rows, a->image.cols, CV_8UC1);
        }

        a->flag  = 1;
    }
    catch ( xiAPIplus_Exception& exp )
    {
        if ( 10 == exp.GetErrorNumber() )
        {
            // Timeout exception.
            a->flag = 2;
        }
        else
        {
            std::cout << ">>> " << a->name 
                      << ": Exception: Ximea API throws exception with error number "
                      << exp.GetErrorNumber() 
                      << ". Thread ends immediately."
                      << std::endl;
            a->flag = 3;
        }
    }

    return (void*)( &(a->flag) );
}

int StereoXiCamera::get_images_mt(cv::Mat &img0, cv::Mat &img1, CameraParams_t &cp0, CameraParams_t &cp1, int n0, int n1)
{
    PROFILER_IN(__PRETTY_FUNCTION__);

    int ret = 0;

    // Create the argument for thd_get_single_image().
    TArg_get_single_image_t args[2];
    std::stringstream ss;

    LOOP_CAMERAS_BEGIN
        ss.str(""); ss.clear(); ss << "thd_get_single_image_" << loopIdx;
        args[loopIdx].name  = ss.str();
        args[loopIdx].cam   = &(mCams[loopIdx]);
        args[loopIdx].flag  = 0;
        args[loopIdx].image = cv::Mat();
        args[loopIdx].grayBuffer = &(mGrayMatBuffer[loopIdx]);
        args[loopIdx].ts[0] = 0;
        args[loopIdx].ts[1] = 0;
    LOOP_CAMERAS_END

    args[CAM_IDX_0].n = n0;
    args[CAM_IDX_1].n = n1;

    pthread_t thds[2];
    void* thdRes[2];
    int s;

    LOOP_CAMERAS_BEGIN
        s = pthread_create(
            thds + loopIdx, NULL, thd_get_single_image, (void*)( args + loopIdx ) );
        
        if ( 0 != s )
        {
            std::cout << "Thread " << args[loopIdx].name << " fails to start." << std::endl;
        }
    LOOP_CAMERAS_END

    LOOP_CAMERAS_BEGIN
        s = pthread_join( thds[loopIdx], (void**)( thdRes + loopIdx ) );

        if ( 0 != s )
        {
            std::cout << "Thread " << args[loopIdx].name << " fails to join." << std::endl;
        }
        else
        {
            // Test use.
            // std::cout << "Thread " << args[loopIdx].name << " joined." << std::endl;
        }
    LOOP_CAMERAS_END

    // // Test use.
    // std::cout << ">>> All threads end. <<<" << std::endl;

    if ( 1 == *( (int*)(thdRes[0]) ) )
    {
        img0 = args[CAM_IDX_0].image;
        cp0.tsSec  = args[CAM_IDX_0].ts[0];
        cp0.tsUSec = args[CAM_IDX_0].ts[1];
    }
    else
    {
        ret = -1;
    }
    
    if ( 1 == *( (int*)(thdRes[1]) ) )
    {
        img1 = args[CAM_IDX_1].image;
        cp1.tsSec  = args[CAM_IDX_1].ts[0];
        cp1.tsUSec = args[CAM_IDX_1].ts[1];
    }
    else
    {
        ret = -1;
    }
    
    // Test use.
    // ret = -1;

    PROFILER_OUT(__PRETTY_FUNCTION__);

    return ret;
}

int StereoXiCamera::get_images(cv::Mat &img0, cv::Mat &img1)
{
    PROFILER_IN(__PRETTY_FUNCTION__);

    try
    {
        img0 = get_single_image(CAM_IDX_0);
        img1 = get_single_image(CAM_IDX_1);

        if ( mGrayMatBuffer[CAM_IDX_0].rows != img0.rows ||
             mGrayMatBuffer[CAM_IDX_0].cols != img0.cols )
        {
            mGrayMatBuffer[CAM_IDX_0] = cv::Mat::zeros(img0.rows, img0.cols, CV_8UC1);
        }

        if ( mGrayMatBuffer[CAM_IDX_1].rows != img1.rows ||
             mGrayMatBuffer[CAM_IDX_1].cols != img1.cols )
        {
            mGrayMatBuffer[CAM_IDX_1] = cv::Mat::zeros(img1.rows, img1.cols, CV_8UC1);
        }
    }
    catch ( xiAPIplus_Exception& exp )
    {
        if ( 10 == exp.GetErrorNumber() )
        {
            // Timeout exception, return.
            return -1;
        }

        EXCEPTION_CAMERA_API(exp);
    }

    PROFILER_OUT(__PRETTY_FUNCTION__);

    return 0;
}

int StereoXiCamera::get_images(cv::Mat &img0, cv::Mat &img1, CameraParams_t &camP0, CameraParams_t &camP1, int n0, int n1)
{
    // if ( 0 != this->get_images(img0, img1) )
    // {
    //     return -1;
    // }

    if ( n0 < 0 || n1 < 0 )
    {
        std::stringstream ss;
        ss << "n0 (" << n0 << ") and n1 (" << n1 << ") must be non-negative numbers.";
        BOOST_THROW_EXCEPTION( bad_argument() << ExceptionInfoString(ss.str()) );
    }

    if ( 0 != this->get_images_mt(img0, img1, camP0, camP1, n0, n1) )
    {
        return -1;
    }

    if ( mRawBalance && false == mIsSelfAdjusting ) {
	// std::cout << "RawBalance \n";
        raw_balance(img0);
        raw_balance(img1);
    }

    try
    {
        put_single_camera_params( mCams[CAM_IDX_0], camP0 );
        put_single_camera_params( mCams[CAM_IDX_1], camP1 );

        if ( true == mCAEAG_IsEnabled )
        {
            if ( false == mIsSelfAdjusting )
            {
                apply_custom_AEAG(img0, img1, camP0, camP1);
            }
        }
        else
        {
            // Evaluate image parameters.
            evaluate_image_parameters(img0, img1, camP0, camP1);
        }
    }
    catch ( xiAPIplus_Exception& exp )
    {
        EXCEPTION_CAMERA_API(exp);
    }

    return 0;
}

void StereoXiCamera::raw_balance(cv::Mat &img) {
    img.convertTo( mRawBalanceBuffer, CV_32FC1 );
    mRawBalanceBuffer = mRawBalanceBuffer.mul( mRawBalanceBR );
    mRawBalanceBuffer.convertTo( img, CV_8UC1 );
}

void StereoXiCamera::put_single_camera_params(xiAPIplusCameraOcv &cam, CameraParams_t &cp)
{
    PROFILER_IN(__PRETTY_FUNCTION__);

    if ( true == cam.IsAutoExposureAutoGain() )
    {
        cp.AEAGEnabled = 1;
    }
    else
    {
        cp.AEAGEnabled = 0;
    }

    cp.AEAGPriority = (xf)( cam.GetAutoExposureAutoGainExposurePriority());

    cp.exposure = (int)( cam.GetExposureTime() );

    cp.gain = (xf)( cam.GetGain() );

    cp.AWBEnabled = ( true == cam.IsWhiteBalanceAuto() ) ? 1 : 0;

    if ( false == mFixedWB )
    {
        cp.AWB_kr = cam.GetWhiteBalanceRed();
        cp.AWB_kg = cam.GetWhiteBalanceGreen();
        cp.AWB_kb = cam.GetWhiteBalanceBlue();
    }
    else
    {
        cp.AWB_kr = mWB_R;
        cp.AWB_kg = mWB_G;
        cp.AWB_kb = mWB_B;
    }
    
    PROFILER_OUT(__PRETTY_FUNCTION__);
}

void StereoXiCamera::stop_acquisition(int waitMS)
{
    try
    {
        // Stop acquisition.
        LOOP_CAMERAS_BEGIN
            mCams[loopIdx].StopAcquisition();
        LOOP_CAMERAS_END
    }
    catch ( xiAPIplus_Exception& exp )
    {
        EXCEPTION_CAMERA_API(exp);
    }

    // cvWaitKey(waitMS); // OpenCV 4.5 has trouble with this.
    std::this_thread::sleep_for(std::chrono::milliseconds(waitMS));
}

void StereoXiCamera::close()
{
    try
    {
        LOOP_CAMERAS_REVERSE_BEGIN
            mCams[loopIdx].Close();
        LOOP_CAMERAS_REVERSE_END
    }
    catch ( xiAPIplus_Exception& exp )
    {
        EXCEPTION_CAMERA_API(exp);
    }
}

void StereoXiCamera::put_sensor_filter_array(int idx, std::string &strFilterArray)
{
    XI_COLOR_FILTER_ARRAY filterArray = mCams[idx].GetSensorColorFilterArray();

    switch ( filterArray )
    {
        case (XI_CFA_NONE):
        {
            strFilterArray = "none";
            break;
        }
        case (XI_CFA_BAYER_RGGB):
        {
            strFilterArray = "bayer_rggb8";
            break;
        }
        case (XI_CFA_CMYG):
        {
            strFilterArray = "cmyg";
            break;
        }
        case (XI_CFA_RGR):
        {
            strFilterArray = "rgr";
            break;
        }
        case (XI_CFA_BAYER_BGGR):
        {
            strFilterArray = "bayer_bggr8";
            break;
        }
        case (XI_CFA_BAYER_GRBG):
        {
            strFilterArray = "bayer_grbg8";
            break;
        }
        case (XI_CFA_BAYER_GBRG):
        {
            strFilterArray = "bayer_gbrg8";
            break;
        }
        default:
        {
            // Should never reach here.
            strFilterArray = "error";
            break;
        }
    }
}

void StereoXiCamera::initialize_raw_balance_matrices(xf cb, xf cr) {
    mRawBalanceBR = cv::Mat::ones( mHeight, mWidth, CV_32FC1 );

    for ( int i = 0; i < mHeight; i += 2 ) {
        for ( int j = 0; j < mWidth; j += 2) {
            mRawBalanceBR.at<float>( i, j ) = cb;
        }
    }

    for ( int i = 1; i < mHeight; i += 2 ) {
        for ( int j = 1; j < mWidth; j += 2) {
            mRawBalanceBR.at<float>( i, j ) = cr;
        }
    }

    mRawBalanceBuffer = cv::Mat::zeros( mHeight, mWidth, CV_32FC1 );

    mRawBalance = true;
}

void StereoXiCamera::prepare_before_opening(void)
{
    // Bandwidth.
    LOOP_CAMERAS_BEGIN
	    mCams[loopIdx].DisableAutoBandwidthCalculation();
    LOOP_CAMERAS_END
}

void StereoXiCamera::open_and_common_settings(void)
{
    // Configure common parameters.
    LOOP_CAMERAS_BEGIN
        char *cstr = new char[mCamSN[loopIdx].length() + 1];
        strcpy(cstr, mCamSN[loopIdx].c_str());

        mCams[loopIdx].OpenBySN(cstr);
        setup_camera_common(mCams[loopIdx]);

        delete [] cstr; cstr = SXC_NULL;
    LOOP_CAMERAS_END

    // Configure synchronization.
    if ( false == mIsExternalTriggered )
    {
        set_stereo_master_trigger();
    }
    else
    {
        set_stereo_external_trigger();
    }

    // External timestamp reset.
    if ( true == mIsExternalTimeStampReset )
    {
        set_external_timestamp_reset();
    }
}

void StereoXiCamera::set_transfer_format_single_camera(xiAPIplusCameraOcv& cam, TransferFormat_t tf)
{
    switch ( tf )
    {
        case TF_COLOR:
        {
            cam.SetImageDataFormat(XI_RGB24);
            break;
        }
        case TF_MONO:
        {
            cam.SetImageDataFormat(XI_MONO8);
            break;
        }
        case TF_RAW:
        {
            cam.SetImageDataFormat(XI_RAW8);
            break;
        }
        default:
        {
            // Should never reach here!
            std::stringstream ss;
            ss << "Unexpected transfer format = " << tf;
            BOOST_THROW_EXCEPTION( exception_base() << ExceptionInfoString(ss.str()) );
        }
    }
}

void StereoXiCamera::setup_camera_common(xiAPIplusCameraOcv& cam)
{
    // Downsample.
    if ( XI_DWN_2x2 == mXi_Downsampling )
    {
        if ( XI_SKIPPING == mXi_DownsamplingType )
        {
            cam.SetDownsamplingType(XI_SKIPPING);
        }
        
        cam.SetDownsampling(XI_DWN_2x2);
    }

    // Set exposure time.
    if ( false == mFixedXiExposureGain )
    {
        cam.SetAutoExposureAutoGainExposurePriority( mXi_AutoGainExposurePriority );
        cam.SetAutoExposureAutoGainTargetLevel(mXi_AutoGainExposureTargetLevel);
        cam.SetAutoExposureTopLimit( mXi_AutoExposureTopLimit );
        cam.SetAutoGainTopLimit( mXi_AutoGainTopLimit );
        cam.EnableAutoExposureAutoGain();
    }
    else
    {
        cam.SetExposureTime(mXi_AutoExposureTopLimit);
        cam.SetGain(mXi_AutoGainTopLimit);
        cam.DisableAutoExposureAutoGain();
    }

	// Enable auto-whitebalance.
    if ( true == mFixedWB )
    {
        cam.DisableWhiteBalanceAuto();
        set_white_balance( cam, mWB_R, mWB_G, mWB_B );
    }
    else
    {
        cam.EnableWhiteBalanceAuto();
    }
    
	// Image format.
	// cam.SetImageDataFormat(XI_RGB24);
    set_transfer_format_single_camera(cam, mTransferFormat);

	// Sensor defects selector.
	cam.SetSensorDefectsCorrectionListSelector(XI_SENS_DEFFECTS_CORR_LIST_SEL_USER0);
	cam.EnableSensorDefectsCorrection();
    cam.SetSensorDefectsCorrectionListSelector(XI_SENS_DEFFECTS_CORR_LIST_SEL_FACTORY);
	cam.EnableSensorDefectsCorrection();

	// Bandwith.
	int cameraDataRate = (int)( mXi_TotalBandwidth / 2.0 * ( 100.0 - mXi_BandwidthMargin ) / 100 );
    mXi_MaxFrameRate = mXi_TotalBandwidth / 2.0 / cameraDataRate;
	cam.SetBandwidthLimit( cameraDataRate );
    cam.SetBandwidthLimitMode( XI_ON );
}

void StereoXiCamera::set_stereo_external_trigger(void)
{
    // Set both cameras to use external trigger.
    LOOP_CAMERAS_BEGIN
        mCams[loopIdx].SetGPISelector(XI_GPI_PORT1);
        mCams[loopIdx].SetGPIMode(XI_GPI_TRIGGER);
        mCams[loopIdx].SetTriggerSource(XI_TRG_EDGE_RISING);
        mCams[loopIdx].SetNextImageTimeout_ms(mXi_NextImageTimeout_ms);
    LOOP_CAMERAS_END
}

void StereoXiCamera::set_stereo_master_trigger(void)
{
    // Set trigger mode on the first camera - as trigger source.
    mCams[CAM_IDX_0].SetTriggerSource(XI_TRG_SOFTWARE);
    mCams[CAM_IDX_0].SetGPOSelector(XI_GPO_PORT1);
    mCams[CAM_IDX_0].SetGPOMode(XI_GPO_EXPOSURE_ACTIVE);

    // Set trigger mode on the second camera - as receiver.
    mCams[CAM_IDX_1].SetGPISelector(XI_GPI_PORT1);
    mCams[CAM_IDX_1].SetGPIMode(XI_GPI_TRIGGER);
    mCams[CAM_IDX_1].SetTriggerSource(XI_TRG_EDGE_RISING);
}

void StereoXiCamera::set_stereo_software_trigger(void)
{
    // Set both camera as software tiggered.
    LOOP_CAMERAS_BEGIN
        mCams[loopIdx].SetTriggerSource(XI_TRG_SOFTWARE);
    LOOP_CAMERAS_END
}

void StereoXiCamera::set_external_timestamp_reset(void)
{
    // Set the external timestamp reset.
    LOOP_CAMERAS_BEGIN
        mCams[loopIdx].SetTimestampResetMode(XI_TS_RST_ARM_PERSIST);
        mCams[loopIdx].SetTimestampResetSource(XI_TS_RST_SRC_GPI_2);
    LOOP_CAMERAS_END

    // mCams[CAM_IDX_0].SetTimeStampResetMode(XI_TS_RST_ARM_PERSIST);
    // mCams[CAM_IDX_0].SetTimeStampResetSource(XI_TS_RST_SRC_GPI_2);
}

int StereoXiCamera::EXPOSURE_MILLISEC(int val)
{
    return val * EXPOSURE_MILLISEC_BASE;
}

int StereoXiCamera::EXPOSURE_FROM_MICROSEC(int val)
{
    return (int)( val / EXPOSURE_MILLISEC_BASE );
}

// ================== Getters and setters. =========================

void StereoXiCamera::set_autogain_exposure_priority(xf val)
{
    if ( val < AUTO_GAIN_EXPOSURE_PRIORITY_MIM ||
         val > AUTO_GAIN_EXPOSURE_PRIORITY_MAX )
    {
        EXCEPTION_ARG_OUT_OF_RANGE(val, AUTO_GAIN_EXPOSURE_PRIORITY_MIM, AUTO_GAIN_EXPOSURE_PRIORITY_MAX);
    }
    else
    {
        mXi_AutoGainExposurePriority = val;
    }
}

xf StereoXiCamera::get_autogain_exposure_priority(void)
{
    return mXi_AutoGainExposurePriority;
}

void StereoXiCamera::set_autogain_exposure_target_level(xf val)
{
    if ( val < AUTO_GAIN_EXPOSURE_TARGET_LEVEL_MIN ||
         val > AUTO_GAIN_EXPOSURE_TARGET_LEVEL_MAX )
    {
        EXCEPTION_ARG_OUT_OF_RANGE(val, AUTO_GAIN_EXPOSURE_TARGET_LEVEL_MIN, AUTO_GAIN_EXPOSURE_TARGET_LEVEL_MAX);
    }
    else
    {
        mXi_AutoGainExposureTargetLevel = val;
    }
}

xf StereoXiCamera::get_autogain_exposure_target_level(void)
{
    return mXi_AutoGainExposureTargetLevel;
}

void StereoXiCamera::set_autoexposure_top_limit(int tLimit)
{
    if ( tLimit < AUTO_EXPOSURE_TOP_LIMIT_MIN ||
         tLimit > AUTO_EXPOSURE_TOP_LIMIT_MAX )
    {
        EXCEPTION_ARG_OUT_OF_RANGE(tLimit, AUTO_EXPOSURE_TOP_LIMIT_MIN, AUTO_EXPOSURE_TOP_LIMIT_MAX);
    }
    else
    {
        mXi_AutoExposureTopLimit = tLimit;
    }
}

int StereoXiCamera::get_autoexposure_top_limit(void)
{
    return mXi_AutoExposureTopLimit;
}

void StereoXiCamera::set_autogain_top_limit(xf tG)
{
    if ( tG < AUTO_GAIN_TOP_LIMIT_MIN ||
         tG > AUTO_GAIN_TOP_LIMIT_MAX )
    {
        EXCEPTION_ARG_OUT_OF_RANGE(tG, AUTO_GAIN_TOP_LIMIT_MIN, AUTO_GAIN_TOP_LIMIT_MAX);
    }
    else
    {
        mXi_AutoGainTopLimit = tG;
    }
}

xf StereoXiCamera::get_autogain_top_limit(void)
{
    return mXi_AutoGainTopLimit;
}

void StereoXiCamera::set_total_bandwidth(int b)
{
    if ( b < TOTAL_BANDWIDTH_MIN ||
         b > TOTAL_BANDWIDTH_MAX )
    {
        EXCEPTION_ARG_OUT_OF_RANGE(b, TOTAL_BANDWIDTH_MIN, TOTAL_BANDWIDTH_MAX);
    }
    else
    {
        mXi_TotalBandwidth = b;
    }
}

int StereoXiCamera::get_total_bandwidth(void)
{
    return mXi_TotalBandwidth;
}

void StereoXiCamera::set_bandwidth_margin(int m)
{
    if ( m < BANDWIDTH_MARGIN_MIN ||
         m > BANDWIDTH_MARGIN_MAX )
    {
        EXCEPTION_ARG_OUT_OF_RANGE(m, BANDWIDTH_MARGIN_MIN, BANDWIDTH_MARGIN_MAX);
    }
    else
    {
        mXi_BandwidthMargin = m;
    }
}

int StereoXiCamera::get_bandwidth_margin(void)
{
    return mXi_BandwidthMargin;
}

xf StereoXiCamera::get_max_frame_rate(void)
{
    return mXi_MaxFrameRate;
}

void StereoXiCamera::set_transfer_format(TransferFormat_t tf)
{
    mTransferFormat = tf;
}

StereoXiCamera::TransferFormat_t StereoXiCamera::get_transfer_format(void)
{
    return mTransferFormat;
}

int StereoXiCamera::get_exposure(void)
{
    return mXi_Exposure;
}

xf StereoXiCamera::get_gain(void)
{
    return mXi_Gain;
}

void StereoXiCamera::put_WB_coefficients(xf& r, xf& g, xf& b)
{
    r = mWB_R;
    g = mWB_G;
    b = mWB_B;
}

void StereoXiCamera::enable_external_trigger(int nextImageTimeout_ms)
{
    mIsExternalTriggered = true;
    mXi_NextImageTimeout_ms = nextImageTimeout_ms;
}

void StereoXiCamera::disable_external_trigger(void)
{
    mIsExternalTriggered = false;
}

bool StereoXiCamera::is_external_triger(void)
{
    return mIsExternalTriggered;
}

int StereoXiCamera::get_next_image_timeout(void)
{
    return mXi_NextImageTimeout_ms;
}

void StereoXiCamera::enable_external_timestamp_reset(void)
{
    mIsExternalTimeStampReset = true;
}

void StereoXiCamera::disable_external_timestamp_reset(void)
{
    mIsExternalTimeStampReset = false;
}

bool StereoXiCamera::is_external_timestamp_reset(void)
{
    return mIsExternalTimeStampReset;
}

void StereoXiCamera::set_self_adjust_trail_loops(int t)
{
    mSelfAdjustNumTrialLoops = t;
}

int StereoXiCamera::get_self_adjust_trail_loops(void)
{
    return mSelfAdjustNumTrialLoops;
}

void StereoXiCamera::set_custom_AEAG(AEAG* aeag)
{
    mCAEAG = aeag;
}

void StereoXiCamera::set_custom_AEAG_target_brightness_level(int level)
{
    mCAEAG_TargetBrightnessLevel = level;
    mCAEAG_TargetBrightnessLevel8Bit = level / 100.0 * 255;
    if ( mCAEAG_TargetBrightnessLevel8Bit > 255 )
    {
        mCAEAG_TargetBrightnessLevel8Bit = 255;
    }
}

int  StereoXiCamera::get_custom_AEGA_target_brightness_level(void)
{
    return mCAEAG_TargetBrightnessLevel;
}

void StereoXiCamera::enable_custom_AEAG(void)
{
    mCAEAG_IsEnabled = true;
}

void StereoXiCamera::disable_custom_AEAG(void)
{
    mCAEAG_IsEnabled = false;
}

bool StereoXiCamera::is_custom_AEAG_enabled(void)
{
    return mCAEAG_IsEnabled;
}

void StereoXiCamera::set_image_parameter_evaluator(AEAG* ipe)
{
    mIPE = ipe;
}

void StereoXiCamera::put_mean_brightness(int* mb)
{
    mb[CAM_IDX_0] = mMeanBrightness[CAM_IDX_0];
    mb[CAM_IDX_1] = mMeanBrightness[CAM_IDX_1];
}

void StereoXiCamera::enable_force_xi_auto_white_balance(void)
{
    mForceXiAutoWhiteBalance = true;
}

void StereoXiCamera::disable_force_xi_auto_white_balance(void)
{
    mForceXiAutoWhiteBalance = false;
}

void StereoXiCamera::enable_fixed_white_balance(xf cr, xf cg, xf cb)
{
    mWB_R = cr;
    mWB_G = cg;
    mWB_B = cb;

    mFixedWB = true;
}

void StereoXiCamera::disable_fixed_white_balance(void)
{
    mFixedWB = false;
}

void StereoXiCamera::enable_debug(void)
{
    mFlagDebug = true;
}

void StereoXiCamera::disable_debug(void)
{
    mFlagDebug = false;
}
