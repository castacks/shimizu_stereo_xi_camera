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

    int parse_launch_parameters(void);
    int prepare(void);
    int synchronize(void);
    int pause(void);
    int destroy(void);

    void set_topic_name_left_image(const std::string& name);
    const std::string& get_topi_name_left_image(void);

    void set_topic_name_right_image(const std::string& name);
    const std::string& get_topi_name_right_image(void);

    void set_out_dir(const std::string& outDir);
    const std::string& get_out_dir(void);

public:
    static const double DEFAULT_AUTO_GAIN_EXPOSURE_PRIORITY     = 0.9;
    static const double DEFAULT_AUTO_GAIN_EXPOSURE_TARGET_LEVEL = 40.0;
    static const int    DEFAULT_AUTO_EXPOSURE_TOP_LIMIT         = 200000;  // Microsecond.
    static const int    DEFAULT_AUTO_GAIN_TOP_LIMIT             = 12;   // dB.
    static const int    DEFAULT_TOTAL_BANDWIDTH                 = 2400;
    static const int    DEFAULT_BANDWIDTH_MARGIN                = 10;
    static const int    DEFAULT_LOOP_RATE                       = 3;
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

private:
    double mAutoGainExposurePriority;
	double mAutoGainExposureTargetLevel;
	int    mAutoExposureTopLimit; // Microsecond.
	int    mAutoGainTopLimit;
	int    mTotalBandwidth;
	int    mBandwidthMargin;
	int    mFlagWriteImage;
	int    mLoopRate;

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