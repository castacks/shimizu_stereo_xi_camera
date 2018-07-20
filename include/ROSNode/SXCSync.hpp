#ifndef __SXC_SYNC_HPP__
#define __SXC_SYNC_HPP__

// ========= Includes for ROS and OpenCV. ===================

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "AEAG/MeanBrightness.hpp"
#include "StereoXiCamera.hpp"

#include "ROSNode/SyncROSNode.hpp"

namespace SRN
{

class SXCSync : public SyncROSNode
{
public:
    SXCSync(const std::string& name);
    virtual ~SXCSync();

    Res_t parse_launch_parameters(void);
    Res_t prepare(void);
    Res_t resume(void);
    Res_t synchronize(void);
    Res_t pause(void);
    Res_t destroy(void);

    void set_topic_name_left_image(const std::string& name);
    const std::string& get_topi_name_left_image(void);

    void set_topic_name_right_image(const std::string& name);
    const std::string& get_topi_name_right_image(void);

    void set_out_dir(const std::string& outDir);
    const std::string& get_out_dir(void);

protected:
    void destroy_members(void);

public:
    static const double DEFAULT_AUTO_GAIN_EXPOSURE_PRIORITY     = 0.9;
    static const double DEFAULT_AUTO_GAIN_EXPOSURE_TARGET_LEVEL = 40.0;
    static const int    DEFAULT_AUTO_EXPOSURE_TOP_LIMIT         = 200000;  // Microsecond.
    static const int    DEFAULT_AUTO_GAIN_TOP_LIMIT             = 12;   // dB.
    static const int    DEFAULT_TOTAL_BANDWIDTH                 = 2400;
    static const int    DEFAULT_BANDWIDTH_MARGIN                = 10;
    static const double DEFAULT_LOOP_RATE                       = 3.0;
    static const int    DEFAULT_NEXT_IMAGE_TIMEOUT_MS           = 1000;
    static const double DEFAULT_CUSTOM_AEAG_PRIORITY            = 0.9;
    static const double DEFAULT_CUSTOM_AEAG_EXPOSURE_TOP_LIMIT  = 200000.0; // Mirosecond.
    static const double DEFAULT_CUSTOM_AEAG_GAIN_TOP_LIMIT      = 12.0;  // dB.
    static const int    DEFAULT_CUSTOM_AEAG_BRIGHTNESS_LEVEL    = 30;    // %.

protected:
    const int CAM_0_IDX;
    const int CAM_1_IDX;

    std::string mTopicNameLeftImage;
    std::string mTopicNameRightImage;

    std::string mOutDir;

    std::string mXiCameraSN[2];

    bool mPrepared;

    image_transport::ImageTransport* mImageTransport;
    image_transport::Publisher* mPublishersImage;

    // The image message to be published.
	sensor_msgs::ImagePtr mMsgImage;

    // The object of stereo camera based on the XIMEA cameras.
    sxc::StereoXiCamera* mStereoXiCamera;

    // The custom AEAG object.
	sxc::MeanBrightness* mMbAEAG;

    // Temporary variables.
    cv::Mat* mCvImages;        // OpenCV Mat array to hold the images.
    sxc::StereoXiCamera::CameraParams_t* mCP; // Camera parameters.
    ros::Time mRosTimeStamp; // ROS time stamp for the header of published ROS image messages.

    std::vector<int> mJpegParams;

    int mNImages;

    // Running rate.
  	ros::Rate* mROSLoopRate;

private:
    double mAutoGainExposurePriority;
	double mAutoGainExposureTargetLevel;
	int    mAutoExposureTopLimit; // Microsecond.
	int    mAutoGainTopLimit;
	int    mTotalBandwidth;
	int    mBandwidthMargin;
	int    mFlagWriteImage;
	double mLoopRate;

	int    mExternalTrigger;
	int    mNextImageTimeout_ms;

	int    mSelfAdjust;

	int    mCustomAEAGEnabled;
	double mCustomAEAGPriority;
	double mCustomAEAGExposureTopLimit;
	double mCustomAEAGGainTopLimit;
	int    mCustomAEAGBrightnessLevel;
};

}

#endif // __SXC_SYNC_HPP__