#ifndef __SXC_SYNC_HPP__
#define __SXC_SYNC_HPP__

// ========= Includes for ROS and OpenCV. ===================

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "AEAG/MeanBrightness.hpp"
#include "StereoXiCamera.hpp"

#include "ROSNode/SyncROSNode.hpp"

// ========== ROS generated headers. ===============

#include "ros_stereo_xi_camera/change_status.h"

namespace SRN
{

class SXCSync : public SyncROSNode
{
public:
    typedef enum
    {
        LAST_STA_UNDEFINED = 0,
        LAST_STA_INIT,
        LAST_STA_PARSE_PARAM,
        LAST_STA_PREPARE,
        LAST_STA_RESUME,
        LAST_STA_SYNC,
        LAST_STA_PAUSE,
        LAST_STA_STOP,
        LAST_STA_DESTROY
    } LastSta_t;

    typedef enum
    {
        LOOP_PARSE_PARAM = 0,
        LOOP_PRPARE,
        LOOP_RESUME,
        LOOP_SYNC,
        LOOP_PAUSE,
        LOOP_STOP,
        LOOP_DESTROY
    } LoopTarget_t;

public:
    SXCSync(const std::string& name);
    virtual ~SXCSync();

    Res_t init(int& argc, char** argv, const std::string& name, uint32_t options = 0);
    Res_t parse_launch_parameters(void);
    Res_t prepare(void);
    Res_t resume(ProcessType_t& pt);
    Res_t synchronize(ProcessType_t& pt);
    Res_t pause(ProcessType_t& pt);
    Res_t stop(void);
    Res_t destroy(void);

    void set_topic_name_left_image(const std::string& name);
    const std::string& get_topi_name_left_image(void);

    void set_topic_name_right_image(const std::string& name);
    const std::string& get_topi_name_right_image(void);

    void set_out_dir(const std::string& outDir);
    const std::string& get_out_dir(void);

    // ROS services.
public:
    bool srv_change_status(
        ros_stereo_xi_camera::change_status::Request &req,
        ros_stereo_xi_camera::change_status::Response &res);

private:
    void destroy_members(void);
    void set_transfer_format(sxc::StereoXiCamera*, const std::string& tf, std::string& encoding);
    void align_cpu_time(ros::Time& cpuTime, const sxc::StereoXiCamera::CameraParams_t& cam);
    void publish_diagnostics( int seq,
        ros::Time& t,
        sxc::StereoXiCamera::CameraParams_t* cpArray,
        int* mbArray );

public:
    static const double DEFAULT_AUTO_GAIN_EXPOSURE_PRIORITY     = 0.9;
    static const double DEFAULT_AUTO_GAIN_EXPOSURE_TARGET_LEVEL = 40.0;
    static const int    DEFAULT_AUTO_EXPOSURE_TOP_LIMIT         = 200000;  // Microsecond.
    static const int    DEFAULT_AUTO_GAIN_TOP_LIMIT             = 12;   // dB.
    static const int    DEFAULT_TOTAL_BANDWIDTH                 = 2400;  // MBits/s.
    static const int    DEFAULT_BANDWIDTH_MARGIN                = 10;    // Percentage.
    static const double DEFAULT_SINGLE_IMAGE_SIZE               = 12.37; // MBytes.
    static const double DEFAULT_LOOP_RATE                       = 3.0;
    static const int    DEFAULT_NEXT_IMAGE_TIMEOUT_MS           = 1000;
    static const double DEFAULT_CUSTOM_AEAG_PRIORITY            = 0.9;
    static const double DEFAULT_CUSTOM_AEAG_EXPOSURE_TOP_LIMIT  = 200000.0; // Mirosecond.
    static const double DEFAULT_CUSTOM_AEAG_GAIN_TOP_LIMIT      = 12.0;  // dB.
    static const int    DEFAULT_CUSTOM_AEAG_BRIGHTNESS_LEVEL    = 30;    // %.
    static const double DEFAULT_CUSTOM_AEAG_CP                  = 2000.0;
    static const double DEFAULT_CUSTOM_AEAG_CD                  = 500.0;
    static const int    DEFAULT_CUSTOM_AEAG_CT                  = 10000; // Mirosecond.
    static const int    DEFAULT_VERBOSE                         = 0;

    static const int    DEFAULT_FORCE_XI_AUTO_WHITE_BALANCE     = 0;

    static const int    SERVICE_REQUEST_CODE_UNDEFINED          = 0;
    static const int    SERVICE_REQUEST_CODE_START              = 1;
    static const int    SERVICE_REQUEST_CODE_PAUSE              = 2;
    static const int    SERVICE_REQUEST_CODE_STOP               = 3;

    const std::string   DEFAULT_TRANSFER_FORMAT;

protected:
    const int CAM_0_IDX;
    const int CAM_1_IDX;

    ros::ServiceServer mROSService;

    std::string mTopicNameLeftImage;
    std::string mTopicNameRightImage;

    std::string mOutDir;

    std::string mXiCameraSN[2];

    LastSta_t mLastStatus;
    LoopTarget_t mLoopTarget;

    int mServiceRequestCode;

    bool mPrepared;

    image_transport::ImageTransport* mImageTransport;
    image_transport::Publisher* mPublishersImage;
    ros::Publisher mTestMsgPublisher;
    ros::Publisher mDiagPublisher;

    // The image message to be published.
	sensor_msgs::ImagePtr mMsgImage;

    // The object of stereo camera based on the XIMEA cameras.
    sxc::StereoXiCamera* mStereoXiCamera;

    // The custom AEAG object.
	sxc::MeanBrightness* mMbAEAG;

    // Image parameter evaluator.
    sxc::MeanBrightness* mIPE;

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
	int    mTotalBandwidth;  // MBits/s.
	int    mBandwidthMargin; // Percentage.
    double mSingleImageSize; // MBytes.
    int    mMinTransferTimeSingleImage; // us.
	int    mFlagWriteImage;
	double mLoopRate;
    std::string mTransferFormat;
    std::string mEncoding;

	int    mExternalTrigger;
	int    mNextImageTimeout_ms;

    int    mExternalTimestampReset;

	int    mSelfAdjust;

	int    mCustomAEAGEnabled;
	double mCustomAEAGPriority;
	double mCustomAEAGExposureTopLimit;
	double mCustomAEAGGainTopLimit;
	int    mCustomAEAGBrightnessLevel;
    double mCustomAEAG_CP;
    double mCustomAEAG_CD;
    double mCustomAEAG_CT;

    int    mForceXiAutoWhiteBalance;

    int    mVerbose;
};

}

#endif // __SXC_SYNC_HPP__