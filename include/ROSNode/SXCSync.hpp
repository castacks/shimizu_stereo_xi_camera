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
// #include "AEAG/MaskedMeanBrightness.hpp"
#include "AEAG/DownSampledMeanBrightness.hpp"
#include "AEAG/CentralMeanBrightness.hpp"
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

    typedef enum
    {
        TRANS_FORMAT_COLOR = 0,
        TRANS_FORMAT_MONO,
        TRANS_FORMAT_RAW
    } TransFormat_t;

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
    /**
     * @param src The input image.
     * @param dstGray The grayscale image.
     * @param dstDS The downsampled image.
     * @param h The height of the downsampled image.
     * @param w The width of the downsampled iamge.
     */
    void convert_downsample_VIO( const cv::Mat& src, cv::Mat& dstGray, cv::Mat& dstDS, const TransFormat_t tf, cv::Size& s );
    void align_cpu_time(ros::Time& cpuTime, const sxc::StereoXiCamera::CameraParams_t& cam);
    void update_number_of_images_needed( const ros::Time& t0, const ros::Time& t1, int& n0, int& n1 );
    void publish_diagnostics( int seq,
        ros::Time& t,
        sxc::StereoXiCamera::CameraParams_t* cpArray,
        int* mbArray );

public:
    static constexpr double DEFAULT_AUTO_GAIN_EXPOSURE_PRIORITY     = 0.9;
    static constexpr double DEFAULT_AUTO_GAIN_EXPOSURE_TARGET_LEVEL = 40.0;
    static constexpr int    DEFAULT_AUTO_EXPOSURE_TOP_LIMIT         = 200000;  // Microsecond.
    static constexpr int    DEFAULT_AUTO_GAIN_TOP_LIMIT             = 12;      // dB.
    static constexpr int    DEFAULT_TOTAL_BANDWIDTH                 = 2400;    // MBits/s.
    static constexpr int    DEFAULT_BANDWIDTH_MARGIN                = 10;      // Percentage.
    static constexpr int    DEFAULT_HARDWARE_DOWNSAMPLING           = 1;       // 1 - Full size, 2 - 1/4 size.
    static constexpr double DEFAULT_SINGLE_IMAGE_SIZE               = 12.37;   // MBytes.
    static constexpr double DEFAULT_LOOP_RATE                       = 3.0;
    static constexpr int    DEFAULT_NEXT_IMAGE_TIMEOUT_MS           = 1000;
    static constexpr double DEFAULT_CUSTOM_AEAG_PRIORITY            = 0.9;
    static constexpr double DEFAULT_CUSTOM_AEAG_EXPOSURE_TOP_LIMIT  = 200000.0; // Mirosecond.
    static constexpr double DEFAULT_CUSTOM_AEAG_GAIN_TOP_LIMIT      = 12.0;  // dB.
    static constexpr int    DEFAULT_CUSTOM_AEAG_BRIGHTNESS_LEVEL    = 30;    // %.
    static constexpr double DEFAULT_CUSTOM_AEAG_EP                  = 0.0;
    static constexpr double DEFAULT_CUSTOM_AEAG_ED                  = 0.0;
    static constexpr double DEFAULT_CUSTOM_AEAG_EI                  = 0.0;
    static constexpr double DEFAULT_CUSTOM_AEAG_GP                  = 0.0;
    static constexpr double DEFAULT_CUSTOM_AEAG_GD                  = 0.0;
    static constexpr double DEFAULT_CUSTOM_AEAG_GI                  = 0.0;
    static constexpr int    DEFAULT_CUSTOM_AEAG_CT                  = 10000; // Mirosecond.
    static constexpr double DEFAULT_CUSTOM_AEAG_DEM                 = 50000; // Mirosecond, delta exposure max.
    static constexpr double DEFAULT_CUSTOM_AEAG_FR                  = 1.35;  // Get from preious AWB.
    static constexpr double DEFAULT_CUSTOM_AEAG_FG                  = 1.00;  // Get from preious AWB.
    static constexpr double DEFAULT_CUSTOM_AEAG_FB                  = 1.39;  // Get from preious AWB.
    static constexpr int    DEFAULT_FIXED_WB                        = 1;
    static constexpr double DEFAULT_WB_R                            = 1.325575;
    static constexpr double DEFAULT_WB_G                            = 1.0;
    static constexpr double DEFAULT_WB_B                            = 2.783976;
    static constexpr int    DEFAULT_VERBOSE                         = 0;

    static constexpr int    DEFAULT_DS_HEIGHT                       = 468; // 640 / 4112 * 3008
    static constexpr int    DEFAULT_DS_WIDTH                        = 640;

    static constexpr int    DEFAULT_FORCE_XI_AUTO_WHITE_BALANCE     = 0;

    static constexpr int    SERVICE_REQUEST_CODE_UNDEFINED          = 0;
    static constexpr int    SERVICE_REQUEST_CODE_START              = 1;
    static constexpr int    SERVICE_REQUEST_CODE_PAUSE              = 2;
    static constexpr int    SERVICE_REQUEST_CODE_STOP               = 3;

    const std::string   DEFAULT_TRANSFER_FORMAT;

protected:
    const int CAM_0_IDX;
    const int CAM_1_IDX;

    ros::ServiceServer mROSService;

    std::string mTopicNameLeftImage;
    std::string mTopicNameRightImage;
    std::string mTopicNameVIOImage0;
    std::string mTopicNameVIOImage1;

    std::string mOutDir;

    std::string mXiCameraSN[2];

    LastSta_t mLastStatus;
    LoopTarget_t mLoopTarget;

    int mServiceRequestCode;

    bool mPrepared;

    image_transport::ImageTransport* mImageTransport;
    image_transport::Publisher* mPublishersImage;
    image_transport::Publisher* mPublishersVIO;

    ros::Publisher mTestMsgPublisher;
    ros::Publisher mDiagPublisher;

    // The image message to be published.
	sensor_msgs::ImagePtr mMsgImage[2];
    sensor_msgs::ImagePtr mMsgVIOImage[2];

    // The object of stereo camera based on the XIMEA cameras.
    sxc::StereoXiCamera* mStereoXiCamera;

    // The custom AEAG object.
	sxc::MeanBrightness* mMbAEAG;

    // Image parameter evaluator.
    sxc::MeanBrightness* mIPE;

    // Temporary variables.
    cv::Mat* mCvImages;        // OpenCV Mat array to hold the images.
    cv::Mat* mCvVIOImages;     // OpenCV Mat array to hold the images for the VIO.
    cv::Mat* mCvVIOImagesDownsampled;

    sxc::StereoXiCamera::CameraParams_t* mCP; // Camera parameters.
    ros::Time mRosTimeStamp; // ROS time stamp for the header of published ROS image messages.

    std::vector<int> mJpegParams;

    int mNImages;

    // Running rate.
  	ros::Rate* mROSLoopRate;

private:
    // 1 for fixed exposure and gain. 
    // The exposure and again values are specified by
    // mAutoGainExposureTopLimit and gain is always 0db.
    int    mFixedExposureGain;
    double mAutoGainExposurePriority;
	double mAutoGainExposureTargetLevel;
	int    mAutoExposureTopLimit; // Microsecond.
	int    mAutoGainTopLimit;
	int    mTotalBandwidth;  // MBits/s.
	int    mBandwidthMargin; // Percentage.
    int    mSensorHeight;    // The original height of the sensor in pixels.
    int    mSensorWidth;     // The original width of the sensor in pixels.
    int    mImageHeight;     // The actual image height.
    int    mImageWidth;      // The actual image width.
    int    mHardwareDownsampling; // 1 - no downsample, 2 - 1/4 imagesize.
    double mSingleImageSize; // MBytes.
    int    mMinTransferTimeSingleImage; // us.
	int    mFlagWriteImage;
	double mLoopRate;
    int    mFrameIntervalUM; // Microsecond.
    std::string mTransferFormat;
    TransFormat_t mTF;

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
    double mCustomAEAG_EP;
    double mCustomAEAG_ED;
    double mCustomAEAG_EI;
    double mCustomAEAG_GP;
    double mCustomAEAG_GD;
    double mCustomAEAG_GI;
    double mCustomAEAG_CT;
    double mCustomAEAG_DEM; // Delta exposure max.
    double mCustomAEAG_FR;  // Factors used for brightness metering on Bayer pattern.
    double mCustomAEAG_FG;
    double mCustomAEAG_FB;
    std::string mCustomAEAG_Mask;

    int    mFixedWB;
    double mWB_R;
    double mWB_G;
    double mWB_B;

    int    mForceXiAutoWhiteBalance;

    int    mVerbose;
    int    mSilent;

    int mNI[2]; // Number of images need to get.

    int mDSHeight;
    int mDSWidth;
    cv::Size mDSSize; // The size of the down sampled image.
};

}

#endif // __SXC_SYNC_HPP__