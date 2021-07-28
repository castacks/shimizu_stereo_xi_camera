#include <iomanip>

#include "ROSNode/SXCSync.hpp"

#include "Profiler/Profiler.hpp"

using namespace cv;
using namespace SRN;

/**
 * \param bw Bandwidth, MBits/s.
 * \param imSize Image size, MBytes.
 * \return Transfer time in us.
 */
static int single_image_transfer_time_ms(int bw, double imSize)
{
    return (int)( imSize * 8 / bw * 1000000 );
}

SXCSync::SXCSync(const std::string& name)
: SyncROSNode(name),
  DEFAULT_TRANSFER_FORMAT("color"),
  CAM_0_IDX(0), CAM_1_IDX(1),
  mTopicNameLeftImage("left/image_raw"), mTopicNameRightImage("right/image_raw"), 
  mTopicNameVIOImage0("cam_0/image"), mTopicNameVIOImage1("cam_1/image"),
  mOutDir("./"),
  mLastStatus(LAST_STA_UNDEFINED),
  mLoopTarget(LOOP_SYNC),
  mServiceRequestCode(SERVICE_REQUEST_CODE_START),
  mPrepared(false),
  mImageTransport(NULL), mPublishersImage(NULL), mPublishersVIO(NULL),
  mStereoXiCamera(NULL),
  mMbAEAG(NULL),
  mIPE(NULL),
  mCvImages(NULL), mCvVIOImages(NULL), mCvVIOImagesDownsampled(NULL), 
  mCP(NULL),
  mNImages(0),
  mROSLoopRate(NULL),
  mFixedExposureGain(0),
  mAutoGainExposurePriority(DEFAULT_AUTO_GAIN_EXPOSURE_PRIORITY),
  mAutoGainExposureTargetLevel(DEFAULT_AUTO_GAIN_EXPOSURE_TARGET_LEVEL),
  mAutoExposureTopLimit(DEFAULT_AUTO_EXPOSURE_TOP_LIMIT),
  mAutoGainTopLimit(DEFAULT_AUTO_GAIN_TOP_LIMIT),
  mTotalBandwidth(DEFAULT_TOTAL_BANDWIDTH),
  mBandwidthMargin(DEFAULT_BANDWIDTH_MARGIN),
  mSensorHeight(3008), mSensorWidth(4112), mImageHeight(mSensorHeight), mImageWidth(mSensorWidth), mHardwareDownsampling(1), 
  mSingleImageSize(DEFAULT_SINGLE_IMAGE_SIZE), mMinTransferTimeSingleImage( single_image_transfer_time_ms(mTotalBandwidth, mSingleImageSize) ),
  mFlagWriteImage(0),
  mLoopRate(DEFAULT_LOOP_RATE), 
  mFrameIntervalUM( int(1000000.0 / DEFAULT_LOOP_RATE) ),
  mPubOnFrames(DEFAULT_PUB_ON_FRAMES), mCountForPub(0),
  mTransferFormat(DEFAULT_TRANSFER_FORMAT), mTF(TRANS_FORMAT_COLOR),
  mEncoding("bgr8"),
  mExternalTrigger(0),
  mNextImageTimeout_ms(DEFAULT_NEXT_IMAGE_TIMEOUT_MS),
  mExternalTimestampReset(0), 
  mSelfAdjust(1),
  mCustomAEAGEnabled(0),
  mCustomAEAGPriority(DEFAULT_CUSTOM_AEAG_PRIORITY),
  mCustomAEAGExposureTopLimit(DEFAULT_CUSTOM_AEAG_EXPOSURE_TOP_LIMIT),
  mCustomAEAGGainTopLimit(DEFAULT_CUSTOM_AEAG_GAIN_TOP_LIMIT),
  mCustomAEAGBrightnessLevel(DEFAULT_CUSTOM_AEAG_BRIGHTNESS_LEVEL),
  mCustomAEAG_FR(DEFAULT_CUSTOM_AEAG_FR), mCustomAEAG_FG(DEFAULT_CUSTOM_AEAG_FG), mCustomAEAG_FB(DEFAULT_CUSTOM_AEAG_FB),
  mCustomAEAG_Mask(""),
  mFixedWB(DEFAULT_FIXED_WB), mWB_R(DEFAULT_WB_R), mWB_G(DEFAULT_WB_G), mWB_B(DEFAULT_WB_B),
  mRawBalance(0),
  mForceXiAutoWhiteBalance(DEFAULT_FORCE_XI_AUTO_WHITE_BALANCE),
  mVerbose(DEFAULT_VERBOSE), mSilent(0),
  mDSHeight(240), mDSWidth(320)
{
    mXiCameraSN[CAM_0_IDX] = "CUCAU1814018";
    mXiCameraSN[CAM_1_IDX] = "CUCAU1814020";

    mNI[CAM_0_IDX] = 1;
    mNI[CAM_1_IDX] = 1;
}

SXCSync::~SXCSync()
{
    destroy_members();

    if ( NULL != mPublishersVIO )
    {
        delete [] mPublishersVIO; mPublishersVIO = NULL;
    }

    if ( NULL != mPublishersImage )
    {
        delete [] mPublishersImage; mPublishersImage = NULL;
    }

    if ( NULL != mImageTransport )
    {
        delete mImageTransport; mImageTransport = NULL;
    }
}

Res_t SXCSync::init(int& argc, char** argv, const std::string& name, uint32_t options)
{
    Res_t ret = RES_OK;

    // Must call parent's init();
    ret = ((SyncROSNode*)this)->init(argc, argv, name, options);

    if ( RES_OK != ret )
    {
        return ret;
    }

    // Topics.
    // =================== Image publishers. ===================
    mImageTransport = new image_transport::ImageTransport((*mpROSNode));

    mPublishersImage = new image_transport::Publisher[2];
    mPublishersImage[CAM_0_IDX] = mImageTransport->advertise(mTopicNameLeftImage,  5);
    mPublishersImage[CAM_1_IDX] = mImageTransport->advertise(mTopicNameRightImage, 5);

    mPublishersVIO = new image_transport::Publisher[2];
    mPublishersVIO[CAM_0_IDX] = mImageTransport->advertise(mTopicNameVIOImage0, 1);
    mPublishersVIO[CAM_1_IDX] = mImageTransport->advertise(mTopicNameVIOImage1, 1);

    mTestMsgPublisher = mpROSNode->advertise<std_msgs::String>("sxc_test_msg", 1000);
    mDiagPublisher    = mpROSNode->advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);

    // Services.
    mROSService = mpROSNode->advertiseService("change_status", &SXCSync::srv_change_status, this);

    return ret;
}

Res_t SXCSync::parse_launch_parameters(void)
{
    // ============ Get the parameters from the launch file. ============== 
	
    std::string pXICameraSN_0 = mXiCameraSN[CAM_0_IDX];
	std::string pXICameraSN_1 = mXiCameraSN[CAM_1_IDX];

    ROSLAUNCH_GET_PARAM((*mpROSNode), "pFixedExposureGain", mFixedExposureGain, 0);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pAutoGainExposurePriority", mAutoGainExposurePriority, DEFAULT_AUTO_GAIN_EXPOSURE_PRIORITY);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pAutoGainExposureTargetLevel", mAutoGainExposureTargetLevel, DEFAULT_AUTO_GAIN_EXPOSURE_TARGET_LEVEL);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pAutoExposureTopLimit", mAutoExposureTopLimit, DEFAULT_AUTO_EXPOSURE_TOP_LIMIT);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pAutoGainTopLimit", mAutoGainTopLimit, DEFAULT_AUTO_GAIN_TOP_LIMIT);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pTotalBandwidth", mTotalBandwidth, DEFAULT_TOTAL_BANDWIDTH);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pBandwidthMargin", mBandwidthMargin, DEFAULT_BANDWIDTH_MARGIN);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pHardwareDownsampling", mHardwareDownsampling, DEFAULT_HARDWARE_DOWNSAMPLING);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pSingleImageSize", mSingleImageSize, DEFAULT_SINGLE_IMAGE_SIZE);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pLoopRate", mLoopRate, DEFAULT_LOOP_RATE);
    mFrameIntervalUM = int( 1000000.0 / mLoopRate );
    ROSLAUNCH_GET_PARAM((*mpROSNode), "pPubOnFrames", mPubOnFrames, DEFAULT_PUB_ON_FRAMES);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pTransferFormat", mTransferFormat, DEFAULT_TRANSFER_FORMAT);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pFlagWriteImage", mFlagWriteImage, 0);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pOutDir", mOutDir, "./");
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pExternalTrigger", mExternalTrigger, 0);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pNextImageTimeout_ms", mNextImageTimeout_ms, DEFAULT_NEXT_IMAGE_TIMEOUT_MS);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pExternalTimestampReset", mExternalTimestampReset, 0);    
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pSelfAdjust", mSelfAdjust, 1)
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pCustomAEAGEnabled", mCustomAEAGEnabled, 0);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pCustomAEAGPriority", mCustomAEAGPriority, DEFAULT_CUSTOM_AEAG_PRIORITY);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pCustomAEAGExposureTopLimit", mCustomAEAGExposureTopLimit, DEFAULT_CUSTOM_AEAG_EXPOSURE_TOP_LIMIT);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pCustomAEAGGainTopLimit", mCustomAEAGGainTopLimit, DEFAULT_CUSTOM_AEAG_GAIN_TOP_LIMIT);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pCustomAEAGBrightnessLevel", mCustomAEAGBrightnessLevel, DEFAULT_CUSTOM_AEAG_BRIGHTNESS_LEVEL);
    ROSLAUNCH_GET_PARAM((*mpROSNode), "pCustomAEAG_EP", mCustomAEAG_EP, DEFAULT_CUSTOM_AEAG_EP);
    ROSLAUNCH_GET_PARAM((*mpROSNode), "pCustomAEAG_ED", mCustomAEAG_ED, DEFAULT_CUSTOM_AEAG_ED);
    ROSLAUNCH_GET_PARAM((*mpROSNode), "pCustomAEAG_EI", mCustomAEAG_EI, DEFAULT_CUSTOM_AEAG_EI);
    ROSLAUNCH_GET_PARAM((*mpROSNode), "pCustomAEAG_GP", mCustomAEAG_GP, DEFAULT_CUSTOM_AEAG_GP);
    ROSLAUNCH_GET_PARAM((*mpROSNode), "pCustomAEAG_GD", mCustomAEAG_GD, DEFAULT_CUSTOM_AEAG_GD);
    ROSLAUNCH_GET_PARAM((*mpROSNode), "pCustomAEAG_GI", mCustomAEAG_GI, DEFAULT_CUSTOM_AEAG_GI);
    ROSLAUNCH_GET_PARAM((*mpROSNode), "pCustomAEAG_CT", mCustomAEAG_CT, DEFAULT_CUSTOM_AEAG_CT);
    ROSLAUNCH_GET_PARAM((*mpROSNode), "pCustomAEAG_DeltaExpMax", mCustomAEAG_DEM, DEFAULT_CUSTOM_AEAG_DEM);
    ROSLAUNCH_GET_PARAM((*mpROSNode), "pCustomAEAG_Mask", mCustomAEAG_Mask, "");
    ROSLAUNCH_GET_PARAM((*mpROSNode), "pCustomAEAG_FR", mCustomAEAG_FR, DEFAULT_CUSTOM_AEAG_FR);
    ROSLAUNCH_GET_PARAM((*mpROSNode), "pCustomAEAG_FG", mCustomAEAG_FG, DEFAULT_CUSTOM_AEAG_FG);
    ROSLAUNCH_GET_PARAM((*mpROSNode), "pCustomAEAG_FB", mCustomAEAG_FB, DEFAULT_CUSTOM_AEAG_FB);
    ROSLAUNCH_GET_PARAM((*mpROSNode), "pFixedWB", mFixedWB, DEFAULT_FIXED_WB);
    ROSLAUNCH_GET_PARAM((*mpROSNode), "pWB_R", mWB_R, DEFAULT_WB_R);
    ROSLAUNCH_GET_PARAM((*mpROSNode), "pWB_G", mWB_G, DEFAULT_WB_G);
    ROSLAUNCH_GET_PARAM((*mpROSNode), "pWB_B", mWB_B, DEFAULT_WB_B);
    ROSLAUNCH_GET_PARAM((*mpROSNode), "pRawBalance", mRawBalance, 0);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pXICameraSN_0", pXICameraSN_0, mXiCameraSN[CAM_0_IDX]);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pXICameraSN_1", pXICameraSN_1, mXiCameraSN[CAM_1_IDX]);

	ROSLAUNCH_GET_PARAM((*mpROSNode), "pForceXiAutoWhiteBalance", mForceXiAutoWhiteBalance, DEFAULT_FORCE_XI_AUTO_WHITE_BALANCE);

    ROSLAUNCH_GET_PARAM((*mpROSNode), "pDSHeight", mDSHeight, DEFAULT_DS_HEIGHT);
    ROSLAUNCH_GET_PARAM((*mpROSNode), "pDSWidth",  mDSWidth,  DEFAULT_DS_WIDTH);
    mDSSize = Size(mDSWidth, mDSHeight);

    ROSLAUNCH_GET_PARAM((*mpROSNode), "pSilent", mSilent, 0);

    mXiCameraSN[CAM_0_IDX] = pXICameraSN_0;
    mXiCameraSN[CAM_1_IDX] = pXICameraSN_1;

    mLastStatus = LAST_STA_PARSE_PARAM;

    if ( 1 == mHardwareDownsampling )
    {
        mImageHeight = mSensorHeight;
        mImageWidth  = mSensorWidth;
    }
    else if ( 2 == mHardwareDownsampling )
    {
        mImageHeight = mSensorHeight / 2;
        mImageWidth  = mSensorWidth  / 2;
    }
    else
    {
        ROS_ERROR("Hardware downsampling selectoin could only be 1 or 2. Get %d.", mHardwareDownsampling);
        return RES_ERROR;
    }

    return RES_OK;
}

void SXCSync::set_transfer_format(sxc::StereoXiCamera* sxCam, const std::string& tf, std::string& encoding)
{
    if ( 0 == tf.compare( "color" ) )
    {
        sxCam->set_transfer_format( sxc::StereoXiCamera::TF_COLOR );
        encoding = "bgr8";
        mTF = TRANS_FORMAT_COLOR;
    }
    else if ( 0 == tf.compare( "mono" ) )
    {
        sxCam->set_transfer_format( sxc::StereoXiCamera::TF_MONO );
        encoding = "mono8";
        mTF = TRANS_FORMAT_MONO;
    }
    else if ( 0 == tf.compare( "raw" ) )
    {
        sxCam->set_transfer_format( sxc::StereoXiCamera::TF_RAW );
        encoding = "bayer_bggr8";
        mTF = TRANS_FORMAT_RAW;
    }
    else
    {
        ROS_ERROR("SXCSync::set_transfer_format: Unexpected tranfser format (%s). Setting default (%s) instead.", 
           tf.c_str(), DEFAULT_TRANSFER_FORMAT.c_str() );

        sxCam->set_transfer_format( sxc::StereoXiCamera::TF_COLOR );
    }
}

void SXCSync::convert_downsample_VIO( const cv::Mat& src, cv::Mat& dstGray, cv::Mat& dstDS, const TransFormat_t tf, cv::Size& s )
{
    // Convert the input image into gray scale image.
    switch ( tf )
    {
        case TRANS_FORMAT_COLOR:
        {
            cv::cvtColor( src, dstGray, cv::COLOR_BGR2GRAY );
            break;
        }
        case TRANS_FORMAT_MONO:
        {
            dstGray = src;
            break;
        }
        case TRANS_FORMAT_RAW:
        {
            cv::cvtColor( src, dstGray, cv::COLOR_BayerRG2GRAY );
            break;
        }
        default:
        {
            ROS_ERROR("Unexpected tf value (%d)", tf);
        }
    }

    // Downsample the image.
    cv::resize( dstGray, dstDS, s, 0.0, 0.0, cv::INTER_LINEAR );
}

Res_t SXCSync::prepare(void)
{
    Res_t res = RES_OK;

    if ( false == mPrepared )
    {
        // Stereo camera object.
        mStereoXiCamera = new sxc::StereoXiCamera(mXiCameraSN[CAM_0_IDX], mXiCameraSN[CAM_1_IDX]);
        
        if ( 1 == mRawBalance ) {
		// std::cout << "RawBalance\n";
		mStereoXiCamera->initialize_raw_balance_matrices(mWB_B, mWB_R);
	}

        // Downsampling.
        if ( 2 == mHardwareDownsampling )
        {
            mStereoXiCamera->enable_downsampling();
        }

        // Trigger selection.
        if ( 1 == mExternalTrigger )
        {
            mStereoXiCamera->enable_external_trigger(mNextImageTimeout_ms);
        }

        // Timestamp reset configuration.
        if ( 1 == mExternalTimestampReset )
        {
            mStereoXiCamera->enable_external_timestamp_reset();
        }

        // Custom AEAG.
        if ( 1 != mFixedExposureGain )
        {
            if ( 1 == mCustomAEAGEnabled )
            {
                if ( 0 == mTransferFormat.compare( "raw" ) )
                {
                    // mMbAEAG = new sxc::MaskedMeanBrightness(mCustomAEAG_Mask);
                    // mMbAEAG = new sxc::DownSampledMeanBrightness(
                    //     mImageWidth, mImageHeight, 100, 100, 
                    //     mCustomAEAG_FR, mCustomAEAG_FG, mCustomAEAG_FB);
                    mMbAEAG = new sxc::CentralMeanBrightness(
                        mImageWidth, mImageHeight, 0.5, 0.5, 100, 100, 
                        mCustomAEAG_FR, mCustomAEAG_FG, mCustomAEAG_FB);
                    ROS_WARN("Use RAW transfer format and custom AEAG.");
                }
                else
                {
                    mMbAEAG = new sxc::MeanBrightness;
                    ROS_WARN("Use the custom AEAG.");
                }
                
                mMbAEAG->set_exposure_top_limit(mCustomAEAGExposureTopLimit);
                mMbAEAG->set_gain_top_limit(sxc::dBToGain(mCustomAEAGGainTopLimit));
                mMbAEAG->set_priority(mCustomAEAGPriority);
                mMbAEAG->set_p(mCustomAEAG_EP, mCustomAEAG_GP);
                mMbAEAG->set_d(mCustomAEAG_ED, mCustomAEAG_GD);
                mMbAEAG->set_i(mCustomAEAG_EI, mCustomAEAG_GI);
                mMbAEAG->set_dem( mCustomAEAG_DEM );

                mStereoXiCamera->set_custom_AEAG(mMbAEAG);
                mStereoXiCamera->set_custom_AEAG_target_brightness_level(mCustomAEAGBrightnessLevel);
                mStereoXiCamera->enable_custom_AEAG();
            }

            mStereoXiCamera->disable_fixed_exposure_gain();
        }
        else
        {
            mStereoXiCamera->enable_fixed_exposure_gain();
        }
        
        // ROS_WARN("After 1 != mFixedExposureGain.");

        // Image parameter evaluator.
        if ( 0 == mTransferFormat.compare( "raw" ) )
        {
            mIPE = new sxc::DownSampledMeanBrightness(mImageWidth, mImageHeight, 100, 100);
            mStereoXiCamera->set_image_parameter_evaluator(mIPE);
        }
        else
        {
            mIPE = new sxc::MeanBrightness;
            mStereoXiCamera->set_image_parameter_evaluator(mIPE);
        }

        // White balance.
        if ( 1 == mFixedWB )
        {
            mStereoXiCamera->enable_fixed_white_balance( mWB_R, mWB_G, mWB_B );
        }
        else
        {
            mStereoXiCamera->disable_fixed_white_balance();
        }
        

        // Prepare with the camera API.
        try
        {
            // Configure the stereo camera.
            mStereoXiCamera->set_autogain_exposure_priority(mAutoGainExposurePriority);
            mStereoXiCamera->set_autogain_exposure_target_level(mAutoGainExposureTargetLevel);
            mStereoXiCamera->set_autoexposure_top_limit(mAutoExposureTopLimit);
            mStereoXiCamera->set_autogain_top_limit(mAutoGainTopLimit);
            mStereoXiCamera->set_total_bandwidth(mTotalBandwidth);
            mStereoXiCamera->set_bandwidth_margin(mBandwidthMargin);

            // Transfer format.
            set_transfer_format(mStereoXiCamera, mTransferFormat, mEncoding);

            // Pre-open, open and configure the stereo camera.
            mStereoXiCamera->open();

            // Self-adjust.
            if ( 1 == mSelfAdjust && 1 != mFixedWB )
            {
                ROS_INFO("Perform self-adjust...");
                mStereoXiCamera->self_adjust(true);
                ROS_INFO("Self-adjust done.");
            }
            else
            {
                ROS_INFO("No self-adjust will be performed.");
            }

            // Get the sensor array.
            std::string strSensorArray;
            mStereoXiCamera->put_sensor_filter_array(0, strSensorArray);
            ROS_INFO("The sensor array string is %s.", strSensorArray.c_str());

            mCvImages = new Mat[2];
            mCvVIOImages = new Mat[2];
            mCvVIOImagesDownsampled = new Mat[2];
            mCP = new sxc::StereoXiCamera::CameraParams_t[2];

            // mImwriteParams.push_back( CV_IMWRITE_PNG_COMPRESSION );
            // mImwriteParams.push_back( 7 );

            mNImages = 0;

            // Running rate.
            mROSLoopRate = new ros::Rate(mLoopRate);

            mPrepared = true;
        }
        catch  ( boost::exception &ex )
        {
            ROS_ERROR("Exception catched.");
            if ( std::string const * expInfoString = boost::get_error_info<sxc::ExceptionInfoString>(ex) )
            {
                ROS_ERROR("%s", expInfoString->c_str());
            }

            std::string diagInfo = boost::diagnostic_information(ex, true);

            ROS_ERROR("%s", diagInfo.c_str());

            res = RES_ERROR;
        }

        // Transfer time.
        mMinTransferTimeSingleImage = single_image_transfer_time_ms( mTotalBandwidth, mSingleImageSize );
        ROS_INFO("Minimum transfer time for a single image is approximatedly %d ms.", mMinTransferTimeSingleImage);

        // Debug use.
        if ( true == mForceXiAutoWhiteBalance )
        {
            mStereoXiCamera->enable_force_xi_auto_white_balance();
        }

        mLastStatus = LAST_STA_PREPARE;

        return res;
    }
    else
    {
        std::cout << "Error: Already prepared." << std::endl;

        mLastStatus = LAST_STA_PREPARE;

        return RES_ERROR;
    }
}

Res_t SXCSync::resume(ProcessType_t& pt)
{
    if ( LAST_STA_PREPARE == mLastStatus || 
         LAST_STA_PAUSE   == mLastStatus )
    {
        // Start acquisition.
        ROS_INFO("%s", "Start acquisition.");
        mStereoXiCamera->start_acquisition();
    }

    // Show status information.
    ROS_INFO("Loop resumed.");

    // ROS spin.
    ros::spinOnce();

    // Sleep.
    mROSLoopRate->sleep();

    mLastStatus = LAST_STA_RESUME;

    pt = (LOOP_RESUME == mLoopTarget) ? PROCESS_CONTINUE : PROCESS_ONCE;

    return RES_OK;
}

Res_t SXCSync::synchronize(ProcessType_t& pt)
{
    PROFILER_IN(__PRETTY_FUNCTION__);

    Res_t res = RES_OK;

    int getImagesRes = 0;
    std::stringstream ss;   // String stream for outputing info.
    std::stringstream testMsgSS;
    std_msgs::String testMsg;

    try
    {
        if ( 0 == mSilent )
        {
            ROS_INFO("nImages = %d", mNImages);
        }

        // Trigger.
        if ( false == mStereoXiCamera->is_external_triger() )
        {
            mStereoXiCamera->software_trigger();
        }

        // Get images.
        getImagesRes = mStereoXiCamera->get_images( 
            mCvImages[CAM_0_IDX], mCvImages[CAM_1_IDX], 
            mCP[CAM_0_IDX], mCP[CAM_1_IDX], mNI[CAM_0_IDX], mNI[CAM_1_IDX] );

        int mb[2];

        if ( 0 != getImagesRes )
        {
            ROS_ERROR("Get images failed");
        }
        else
        {
            step_count_for_pub();

            ros::Time imageTS;

            // Convert the image into ROS image message.
            LOOP_CAMERAS_BEGIN
                // Clear the temporary sting stream.
                ss.flush();	ss.str(""); ss.clear();
                ss << mOutDir << "/" << mNImages << "_" << loopIdx;

                if ( true == mVerbose )
                {
                    ROS_INFO( "%s", ss.str().c_str() );
                }

                // Save the captured image to file system.
                if ( 1 == mFlagWriteImage )
                {
                    // std::string yamlFilename = ss.str() + ".yaml";
                    std::string imgFilename = ss.str() + ".png";

                    // FileStorage cfFS(yamlFilename, FileStorage::WRITE);
                    // cfFS << "frame" << nImages << "image_id" << loopIdx << "raw_data" << cvImages[loopIdx];
                    // imwrite(imgFilename, mCvImages[loopIdx], mImwriteParams); // OpenCV 4.5 makes CV_IMWRITE_PNG_COMPRESSION obsolete.
                    imwrite(imgFilename, mCvImages[loopIdx] );
                }
                
                if ( true == mVerbose )
                {
                    ROS_INFO( "Camera %d captured image (%d, %d, type = %d). AEAG %d, AEAGP %.2f, exp %.3f ms, gain %.1f dB.", 
                        loopIdx, mCvImages[loopIdx].rows, mCvImages[loopIdx].cols, mCvImages[loopIdx].type(),
                        mCP[loopIdx].AEAGEnabled, mCP[loopIdx].AEAGPriority, mCP[loopIdx].exposure / 1000.0, mCP[loopIdx].gain );
                }
                else if ( 0 == mSilent )
                {
                    ROS_INFO( "Cam %d, E %.3f ms, G %.1f dB.", loopIdx, mCP[loopIdx].exposure / 1000.0, mCP[loopIdx].gain );
                }

                // Create the downsampled version.
                convert_downsample_VIO( mCvImages[loopIdx], mCvVIOImages[loopIdx], mCvVIOImagesDownsampled[loopIdx],
                    mTF, mDSSize );

                // Publish images.
                PROFILER_IN("cv_bridge::CvImage");
                mMsgImage[loopIdx]    = cv_bridge::CvImage(std_msgs::Header(), mEncoding, mCvImages[loopIdx]).toImageMsg();
                mMsgVIOImage[loopIdx] = cv_bridge::CvImage(std_msgs::Header(), "mono8", mCvVIOImagesDownsampled[loopIdx]).toImageMsg();
                PROFILER_OUT("cv_bridge::CvImage");

                // header.seq is not supposed to be filled by user.
                // https://answers.ros.org/question/55126/why-does-ros-overwrite-my-sequence-number/
                // mMsgImage[loopIdx]->header.seq = mNImages;
                
                // Prepare the time stamp for the header.
                imageTS = ros::Time::now();
                if ( 1 == mExternalTimestampReset )
                {
                    align_cpu_time( imageTS, mCP[loopIdx] );

                    // Check the timestamp.
                    if ( mCP[loopIdx].tsSec > 0 )
                    {
                        ROS_WARN("Stereo camera %d missed %d PPS signals.", loopIdx, mCP[loopIdx].tsSec);
                    }
                }

                mMsgImage[loopIdx]->header.stamp    = imageTS;
                mMsgVIOImage[loopIdx]->header.stamp = imageTS;

                // std::cout << loopIdx << ":" << imageTS.sec << "." << std::setfill('0') << std::setw(9) << imageTS.nsec << std::endl;
            LOOP_CAMERAS_END

            // Figure out if we need modify the mNI variable.
            update_number_of_images_needed( 
                mMsgImage[CAM_0_IDX]->header.stamp, mMsgImage[CAM_1_IDX]->header.stamp,
                mNI[CAM_0_IDX], mNI[CAM_1_IDX] );

            // std::cout << "mNI = { " << mNI[0] << ", " << mNI[1] << " }. " << "mFrameIntervalUM = " << mFrameIntervalUM << std::endl;

            if ( mNI[0] != 1 || mNI[1] != 1 )
            {
                ROS_WARN( "mNI: { %d, %d }. Time stamps: %d.%09d, %d.%09d. Frame interval: %d um. ", 
                    mNI[0], mNI[1], 
                    mMsgImage[CAM_0_IDX]->header.stamp.sec, mMsgImage[CAM_0_IDX]->header.stamp.nsec, 
                    mMsgImage[CAM_1_IDX]->header.stamp.sec, mMsgImage[CAM_1_IDX]->header.stamp.nsec,
                    mFrameIntervalUM );
            }

            // This is due to the fact that some downstream programs require the timestamps of
            // the images making a stereo pair to have exactly the same timestamps.
            mMsgVIOImage[1]->header.stamp = mMsgVIOImage[0]->header.stamp;

            if ( should_publish() ) {
                LOOP_CAMERAS_BEGIN
                    PROFILER_IN("ImagePublishing");
                    mPublishersImage[loopIdx].publish(mMsgImage[loopIdx]);
                    mPublishersVIO[loopIdx].publish(mMsgVIOImage[loopIdx]);
                    PROFILER_OUT("ImagePublishing");

                    if ( true == mVerbose )
                    {
                        ROS_INFO("%s", "Message published.");
                    }
                LOOP_CAMERAS_END
            }

            mStereoXiCamera->put_mean_brightness(mb);

            if ( should_publish() ) {
                testMsgSS << "{" << std::endl
                        << "\t\"seq\": " << mNImages << "," << std::endl
                        << "\t\"cams\": [" << std::endl
                        << "\t\t{ \"idx\": 0," << std::endl
                        << "\t\t\"tsSec\": " << mCP[CAM_0_IDX].tsSec << "," << std::endl
                        << "\t\t\"tsUSec\": " << mCP[CAM_0_IDX].tsUSec << "," << std::endl
                        << "\t\t\"exp\": " << mCP[CAM_0_IDX].exposure / 1000.0 << "," << std::endl
                        << "\t\t\"gain\": " << mCP[CAM_0_IDX].gain << "," << std::endl
                        << "\t\t\"mb\": " << mb[CAM_0_IDX] << "," << std::endl
                        << "\t\t\"wbKr\": " << mCP[CAM_0_IDX].AWB_kr << "," << std::endl
                        << "\t\t\"wbKg\": " << mCP[CAM_0_IDX].AWB_kg << "," << std::endl
                        << "\t\t\"wbKb\": " << mCP[CAM_0_IDX].AWB_kb << "" << std::endl
                        << "\t}," << std::endl
                        << "\t\t{ \"idx\": 1," << std::endl
                        << "\t\t\"tsSec\": " << mCP[CAM_1_IDX].tsSec << "," << std::endl
                        << "\t\t\"tsUSec\": " << mCP[CAM_1_IDX].tsUSec << "," << std::endl
                        << "\t\t\"exp\": " << mCP[CAM_1_IDX].exposure / 1000.0 << "," << std::endl
                        << "\t\t\"gain\": " << mCP[CAM_1_IDX].gain << "," << std::endl
                        << "\t\t\"mb\": " << mb[CAM_1_IDX] << "," << std::endl
                        << "\t\t\"wbKr\": " << mCP[CAM_1_IDX].AWB_kr << "," << std::endl
                        << "\t\t\"wbKg\": " << mCP[CAM_1_IDX].AWB_kg << "," << std::endl
                        << "\t\t\"wbKb\": " << mCP[CAM_1_IDX].AWB_kb << "" << std::endl
                        << "\t} ]" << std::endl
                        << "}" << std::endl;
                testMsg.data = testMsgSS.str();
                mTestMsgPublisher.publish(testMsg);
            }
        }

        mRosTimeStamp = ros::Time::now();
        publish_diagnostics(mNImages, mRosTimeStamp, mCP, mb);

        // Debug use.
        if ( true == mForceXiAutoWhiteBalance )
        {
            ROS_INFO( "ForceXiAutoWhiteBalance: krgb0 [%f, %f, %f], krgb1 [%f, %f, %f].", 
                mCP[CAM_0_IDX].AWB_kr, mCP[CAM_0_IDX].AWB_kg, mCP[CAM_0_IDX].AWB_kb, 
                mCP[CAM_1_IDX].AWB_kr, mCP[CAM_1_IDX].AWB_kg, mCP[CAM_1_IDX].AWB_kb );
        }

        // ROS spin.
        ros::spinOnce();

        // Sleep.
        PROFILER_IN("ROSSleep");
        mROSLoopRate->sleep();
        PROFILER_OUT("ROSSleep");

        reset_count_for_pub_if_published();
        mNImages++;
    }
    catch ( boost::exception &ex )
    {
        ROS_ERROR("Exception catched.");
        if ( std::string const * expInfoString = boost::get_error_info<sxc::ExceptionInfoString>(ex) )
        {
            ROS_ERROR("%s", expInfoString->c_str());
        }

        std::string diagInfo = boost::diagnostic_information(ex, true);

        ROS_ERROR("%s", diagInfo.c_str());

        res = RES_ERROR;
    }

    mLastStatus = LAST_STA_SYNC;

    pt = (mLoopTarget == LOOP_SYNC) ? PROCESS_CONTINUE : PROCESS_ONCE;

    PROFILER_OUT(__PRETTY_FUNCTION__);

    return res;
}

/*
 * Assumptions regarding the time synchronization between the camera and the cpu.
 * (1) Ths PPS signal is fired at exactly integer second timestamp, such as 1.0s, 2.0s. PPS signal
 * is not possible to be fired at timestamps like 1.1s, 2.5s, or 3.9s.
 * (2) An image data must be received and processed within 3 PPS signals. That means the maximum
 * time lag between a camera trigger and the timestamp this very image is received and processed by
 * the cpu must be less than 2s.
 * (3) Once there is ambiguity, choose the nearest timestamp measured from the cpu timestamp.
 *
 * The input argument cpuTime will be altered according to argument cam and member variable
 * mMinTransferTimeSingleImage.
*/
void SXCSync::align_cpu_time(ros::Time& cpuTime, const sxc::StereoXiCamera::CameraParams_t& cam)
{
    // Convert cpuTime.nsec into microsecond.
    DWORD cpuMSec = (DWORD)( cpuTime.nsec / 1000 );
    if ( cpuMSec > cam.tsUSec + mMinTransferTimeSingleImage + cam.exposure )
    {
        // Reveived data is recorded in the same sencod.
        cpuTime.nsec = cam.tsUSec * 1000;
    }
    else
    {
        // Received data is recorded in the previous second.
        // Here we consider the assumption (3).
        cpuTime.sec -= 1;
        cpuTime.nsec = cam.tsUSec * 1000;
    }

    // Handle the second field recorded in the data.
    cpuTime.sec -= cam.tsSec;
}

void SXCSync::update_number_of_images_needed( const ros::Time& t0, const ros::Time& t1, int& n0, int& n1 )
{
    int diff = static_cast<int>( 
        ( t1.sec + t1.nsec / 1e9 - t0.sec - t0.nsec / 1e9 ) * 1000000 );
    
    // std::cout << "diff = " << diff << std::endl;

    if ( diff >= mFrameIntervalUM )
    {
        // 0-camera's image is older. 0-camera needs to receive more images.
        n0 = 1 + diff / mFrameIntervalUM;
        n1 = 1;
    }
    else if ( diff <= -mFrameIntervalUM )
    {
        // 0-camera's image is newer. 1-camera needs to receive more images.
        n0 = 1;
        n1 = 1 + (-diff) / mFrameIntervalUM;
    }
    else
    {
        n0 = 1;
        n1 = 1;
    }
}

void SXCSync::publish_diagnostics( int seq,
        ros::Time& t,
        sxc::StereoXiCamera::CameraParams_t* cpArray,
        int* mbArray )
{
    // Diagnostics.
    diagnostic_msgs::DiagnosticArray da;
    diagnostic_msgs::DiagnosticStatus ds;
    std::stringstream ssDiag;

    ds.level       = diagnostic_msgs::DiagnosticStatus::OK;
    ds.name        = "stereo ximea camera";
    ds.message     = "Diagnostic message.";
    ds.hardware_id = "sxc";
    diagnostic_msgs::KeyValue kv;

    // Key-value pairs.
    ssDiag << seq;
    kv.key   = "seq";
    kv.value = ssDiag.str();
    ds.values.push_back(kv);

    ssDiag.str(""); ssDiag.clear();
    ssDiag << cpArray[CAM_0_IDX].exposure / 1000.0 << "," 
           << cpArray[CAM_1_IDX].exposure / 1000.0;
    kv.key   = "exp";
    kv.value = ssDiag.str();
    ds.values.push_back(kv);

    ssDiag.str(""); ssDiag.clear();
    ssDiag << cpArray[CAM_0_IDX].gain / 1000.0 << "," 
           << cpArray[CAM_1_IDX].gain / 1000.0;
    kv.key   = "gain";
    kv.value = ssDiag.str();
    ds.values.push_back(kv);

    ssDiag.str(""); ssDiag.clear();
    ssDiag << mbArray[CAM_0_IDX] << "," 
           << mbArray[CAM_1_IDX];
    kv.key   = "mb";
    kv.value = ssDiag.str();
    ds.values.push_back(kv);

    da.status.push_back(ds);

    da.header.seq   = seq;
    da.header.stamp = t;

    mDiagPublisher.publish(da);
}

Res_t SXCSync::pause(ProcessType_t& pt)
{
    if ( LAST_STA_SYNC   == mLastStatus ||
         LAST_STA_RESUME == mLastStatus )
    {
        // Stop acquisition.
        ROS_INFO("Stop acquisition.");
        mStereoXiCamera->stop_acquisition();
    }
    
    // Show pasue status.
    ROS_INFO("Loop paused.");

    // ROS spin.
    ros::spinOnce();

    // Sleep.
    mROSLoopRate->sleep();

    mLastStatus = LAST_STA_PAUSE;

    pt = (mLoopTarget == LOOP_PAUSE) ? PROCESS_CONTINUE : PROCESS_ONCE;

    return RES_OK;
}

Res_t SXCSync::stop(void)
{
    if ( LAST_STA_PAUSE  == mLastStatus ||
         LAST_STA_SYNC   == mLastStatus ||
         LAST_STA_RESUME == mLastStatus )
    {
        if ( LAST_STA_PAUSE != mLastStatus )
        {
            // Stop acquisition.
            ROS_INFO("Stop acquisition.");
            mStereoXiCamera->stop_acquisition();
        }
    }

    mLastStatus = LAST_STA_STOP;

    return RES_OK;
}

Res_t SXCSync::destroy(void)
{
    Res_t ret = RES_OK;

    if ( LAST_STA_STOP == mLastStatus ||
         LAST_STA_INIT == mLastStatus )
    {
        // Close.
        mStereoXiCamera->close();
        ROS_INFO("Stereo camera closed.");

        destroy_members();

        mPrepared = false;
    }
    else
    {
        ROS_ERROR("Try to destroy more than onec.");

        ret = RES_ERROR;
    }

    mLastStatus = LAST_STA_DESTROY;

    return ret;
}

void SXCSync::destroy_members(void)
{
    if ( NULL != mROSLoopRate )
    {
        delete mROSLoopRate; mROSLoopRate = NULL;
    }

    if ( NULL != mCP )
    {
        delete [] mCP; mCP = NULL;
    }

    if ( NULL != mCvVIOImagesDownsampled )
    {
        delete [] mCvVIOImagesDownsampled; mCvVIOImagesDownsampled = NULL;
    }

    if ( NULL != mCvVIOImages )
    {
        delete [] mCvVIOImages; mCvVIOImages = NULL;
    }

    if ( NULL != mCvImages )
    {
        delete [] mCvImages; mCvImages = NULL;
    }

    if ( NULL != mIPE )
    {
        delete mIPE; mIPE = NULL;
    }

    if ( NULL != mMbAEAG )
    {
        delete mMbAEAG; mMbAEAG = NULL;
    }

    if ( NULL != mStereoXiCamera )
    {
        delete mStereoXiCamera; mStereoXiCamera = NULL;
    }
}

bool SXCSync::srv_change_status(
        ros_stereo_xi_camera::change_status::Request &req,
        ros_stereo_xi_camera::change_status::Response &res)
{
    bool ret = true;
    int s = (int)( req.s );

    switch ( s )
    {
        case SERVICE_REQUEST_CODE_START:
        {
            mServiceRequestCode = SERVICE_REQUEST_CODE_START;
            mLoopTarget = LOOP_SYNC;
            mIsLooping = true;
            ROS_INFO("Service request START received.");
            break;
        }
        case SERVICE_REQUEST_CODE_PAUSE:
        {
            mServiceRequestCode = SERVICE_REQUEST_CODE_STOP;
            mLoopTarget = LOOP_PAUSE;
            mIsLooping = true;
            break;
        }
        case SERVICE_REQUEST_CODE_STOP:
        {
            mServiceRequestCode = SERVICE_REQUEST_CODE_STOP;
            mLoopTarget = LOOP_STOP;
            mIsLooping = false;
            ROS_INFO("Service request STOP received.");
            break;
        }
        default:
        {
            mServiceRequestCode = SERVICE_REQUEST_CODE_UNDEFINED;
            ROS_ERROR("Wrong service request code %d.", s);
            ret = false;
            break;
        }
    }

    // Copy the request code to response code.
    res.r = req.s;

    return ret;
}

void SXCSync::set_topic_name_left_image(const std::string& name)
{
    mTopicNameLeftImage = name;
}

const std::string& SXCSync::get_topi_name_left_image(void)
{
    return mTopicNameLeftImage;
}

void SXCSync::set_topic_name_right_image(const std::string& name)
{
    mTopicNameRightImage = name;
}

const std::string& SXCSync::get_topi_name_right_image(void)
{
    return mTopicNameRightImage;
}

void SXCSync::set_out_dir(const std::string& outDir)
{
    mOutDir = outDir;
}

const std::string& SXCSync::get_out_dir(void)
{
    return mOutDir;
}
