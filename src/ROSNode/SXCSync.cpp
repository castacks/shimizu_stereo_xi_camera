#include "ROSNode/SXCSync.hpp"

using namespace cv;
using namespace SRN;

SXCSync::SXCSync(const std::string& name)
: SyncROSNode(name),
  CAM_0_IDX(0), CAM_1_IDX(1),
  mTopicNameLeftImage("left/image_raw"), mTopicNameRightImage("right/image_raw"), 
  mOutDir("./"),
  mAutoGainExposurePriority(DEFAULT_AUTO_GAIN_EXPOSURE_PRIORITY),
  mAutoGainExposureTargetLevel(DEFAULT_AUTO_GAIN_EXPOSURE_TARGET_LEVEL),
  mAutoExposureTopLimit(DEFAULT_AUTO_EXPOSURE_TOP_LIMIT),
  mAutoGainTopLimit(DEFAULT_AUTO_GAIN_TOP_LIMIT),
  mTotalBandwidth(DEFAULT_TOTAL_BANDWIDTH),
  mBandwidthMargin(DEFAULT_BANDWIDTH_MARGIN),
  mFlagWriteImage(0),
  mLoopRate(DEFAULT_LOOP_RATE),
  mExternalTrigger(0),
  mNextImageTimeout_ms(DEFAULT_NEXT_IMAGE_TIMEOUT_MS),
  mSelfAdjust(1),
  mCustomAEAGEnabled(0),
  mCustomAEAGPriority(DEFAULT_CUSTOM_AEAG_PRIORITY),
  mCustomAEAGExposureTopLimit(DEFAULT_CUSTOM_AEAG_EXPOSURE_TOP_LIMIT),
  mCustomAEAGGainTopLimit(DEFAULT_CUSTOM_AEAG_GAIN_TOP_LIMIT),
  mCustomAEAGBrightnessLevel(DEFAULT_CUSTOM_AEAG_BRIGHTNESS_LEVEL)
{
    mXiCameraSN[CAM_0_IDX] = "CUCAU1814018";
    mXiCameraSN[CAM_1_IDX] = "CUCAU1814020";
}

SXCSync::~SXCSync()
{

}

int SXCSync::parse_launch_parameters(void)
{
    // ============ Get the parameters from the launch file. ============== 
	
    std::string pXICameraSN_0 = mXiCameraSN[CAM_0_IDX];
	std::string pXICameraSN_1 = mXiCameraSN[CAM_1_IDX];

	ROSLAUNCH_GET_PARAM((*mpROSNode), "pAutoGainExposurePriority", mAutoGainExposurePriority, DEFAULT_AUTO_GAIN_EXPOSURE_PRIORITY);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pAutoGainExposureTargetLevel", mAutoGainExposureTargetLevel, DEFAULT_AUTO_GAIN_EXPOSURE_TARGET_LEVEL);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pAutoExposureTopLimit", mAutoExposureTopLimit, DEFAULT_AUTO_EXPOSURE_TOP_LIMIT);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pAutoGainTopLimit", mAutoGainTopLimit, DEFAULT_AUTO_GAIN_TOP_LIMIT);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pTotalBandwidth", mTotalBandwidth, DEFAULT_TOTAL_BANDWIDTH);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pBandwidthMargin", mBandwidthMargin, DEFAULT_BANDWIDTH_MARGIN);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pLoopRate", mLoopRate, DEFAULT_LOOP_RATE);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pFlagWriteImage", mFlagWriteImage, 0);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pOutDir", mOutDir, "./");
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pExternalTrigger", mExternalTrigger, 0);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pNextImageTimeout_ms", mNextImageTimeout_ms, DEFAULT_NEXT_IMAGE_TIMEOUT_MS);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pSelfAdjust", mSelfAdjust, 1)
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pCustomAEAGEnabled", mCustomAEAGEnabled, 0);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pCustomAEAGPriority", mCustomAEAGPriority, DEFAULT_CUSTOM_AEAG_PRIORITY);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pCustomAEAGExposureTopLimit", mCustomAEAGExposureTopLimit, DEFAULT_CUSTOM_AEAG_EXPOSURE_TOP_LIMIT);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pCustomAEAGGainTopLimit", mCustomAEAGGainTopLimit, DEFAULT_CUSTOM_AEAG_GAIN_TOP_LIMIT);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pCustomAEAGBrightnessLevel", mCustomAEAGBrightnessLevel, DEFAULT_CUSTOM_AEAG_BRIGHTNESS_LEVEL);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pXICameraSN_0", pXICameraSN_0, mXiCameraSN[CAM_0_IDX]);
	ROSLAUNCH_GET_PARAM((*mpROSNode), "pXICameraSN_1", pXICameraSN_1, mXiCameraSN[CAM_1_IDX]);

    mXiCameraSN[CAM_0_IDX] = pXICameraSN_0;
    mXiCameraSN[CAM_1_IDX] = pXICameraSN_1;

    return 0;
}

int SXCSync::prepare(void)
{
    // =================== Image publishers. ===================
	image_transport::ImageTransport imageTransport(nodeHandle);

	image_transport::Publisher publishersImage[2] = { 
    imageTransport.advertise(TOPIC_NAME_LEFT_IMAGE, 1), 
    imageTransport.advertise(TOPIC_NAME_RIGHT_IMAGE, 1) };
    return 0;
}

int SXCSync::synchronize(void)
{
    return 0;
}

int SXCSync::pause(void)
{
    return 0;
}

int SXCSync::destroy(void)
{
    return 0;
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