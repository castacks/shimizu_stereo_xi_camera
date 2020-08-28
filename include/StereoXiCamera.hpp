#ifndef __STEREOXICAMERA_H__
#define __STEREOXICAMERA_H__

// =============== C headers. ====================

/* No hearders specified. */

// ============ C++ standard headers. ============

#include <exception>
#include <string>
#include <vector>

// =========== System headers. =================

#include <boost/exception/all.hpp>
#include <boost/shared_ptr.hpp>

// ============ Field specific headers. ========

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// ========== Application headers. ==============

#include "sxc_common.hpp"

#include "AEAG/AEAG.hpp"
#include "xiApiPlusOcv.hpp"

// ============ Macros. ========================

#define SXC_NULL (0)

#define N_XI_C 2

#define LOOP_CAMERAS_BEGIN \
	for( int loopIdx = 0; loopIdx < N_XI_C; loopIdx++ )\
	{

#define LOOP_CAMERAS_END \
	}

#define LOOP_CAMERAS_REVERSE_BEGIN \
	for( int loopIdx = N_XI_C - 1; loopIdx >= 0; loopIdx-- )\
	{

#define LOOP_CAMERAS_REVERSE_END \
	}

#define EXCEPTION_ARG_OUT_OF_RANGE(v, minV, maxV) \
    {\
        std::stringstream v##_ss;\
        v##_ss << "Argument out of range, " \
               << #v << " = " << v \
               << ", [" << minV << ", " << maxV << "]. "\
               << "Value not changed.";\
        BOOST_THROW_EXCEPTION( argument_out_of_range() << ExceptionInfoString(v##_ss.str()) );\
    }

#define EXCEPTION_ARG_NULL(v) \
    {\
        std::stringstream v##_ss;\
        v##_ss << "Argument " \
               << #v << " is NULL.";\
        BOOST_THROW_EXCEPTION( argument_null() << ExceptionInfoString(v##_ss.str()) );\
    }

#define CAMERA_EXCEPTION_DESCRIPTION_BUFFER_SIZE (1024)
#define EXCEPTION_CAMERA_API(camEx) \
    {\
        char camEx##_buffer[CAMERA_EXCEPTION_DESCRIPTION_BUFFER_SIZE];\
        std::stringstream camEx##_ss;\
        \
        camEx.GetDescription(camEx##_buffer, CAMERA_EXCEPTION_DESCRIPTION_BUFFER_SIZE);\
        \
        camEx##_ss << "Camera API throws exception. Error number: "\
                   << camEx.GetErrorNumber()\
                   << ", with description \"" << camEx##_buffer << "\"";\
        \
        BOOST_THROW_EXCEPTION( camera_api_exception() << ExceptionInfoString(camEx##_ss.str()) );\
    }

#define EXCEPTION_BAD_HARDWARE_RESPONSE(s) \
    {\
        std::stringstream badHwRsp_ss;\
        badHwRsp_ss << "Bad hardware response" << s;\
        \
        BOOST_THROW_EXCEPTION( bad_hardware_response() << ExceptionInfoString(badHwRsp_ss.str()) );\
    }

// ============ File-wise or global variables. ===========

/* No variables declared or initialized. */

// ============= Class definitions. ======================

namespace sxc {

struct exception_base        : virtual std::exception, virtual boost::exception { };
struct bad_argument          : virtual exception_base { };
struct argument_out_of_range : virtual bad_argument { };
struct argument_null         : virtual bad_argument { };
struct hardware_exception    : virtual exception_base { };
struct camera_api_exception  : virtual hardware_exception { };
struct bad_hardware_response : virtual hardware_exception { };

typedef boost::error_info<struct tag_info_string, std::string> ExceptionInfoString;

class StereoXiCamera 
{
public:
    typedef enum
    {
        TF_COLOR = 0,
        TF_MONO,
        TF_RAW
    } TransferFormat_t;

    typedef struct CameraParams
    {
        int AEAGEnabled;  // 1 for enabled.
        xf  AEAGPriority;
        int exposure;     // Milliseconds. Microseconds!
        xf  gain;         // db.
        int AWBEnabled;   // 1 for enabled.
        xf  AWB_kr;       // Auto white balance, read coefficient.
        xf  AWB_kg;       // Auto white balance, green coefficient.
        xf  AWB_kb;       // Auto white balance, blue coefficient.

        DWORD tsSec;
        DWORD tsUSec;
    } CameraParams_t;

public:
    StereoXiCamera(std::string &camSN0, std::string &camSN1);
    ~StereoXiCamera();

    void open();
    void self_adjust(bool verbose = false);
    void start_acquisition(int waitMS = 500);

    void software_trigger(bool both = false);
    /**
     * @param n0 Number of images to get for the first camera.
     * @param n1 Number of images to get for the second camera.
     * @return 0 if succeed.
     */
    int get_images(cv::Mat &img0, cv::Mat &img1, CameraParams_t &camP0, CameraParams_t &camP1, int n0=1, int n1=1);
    void stop_acquisition(int waitMS = 500);
    void close();

    void put_sensor_filter_array(int idx, std::string &strFilterArray);

    void initialize_raw_balance_matrices(xf cb, xf cr);

    // Getters and setters.
    void enable_downsampling(void);
    void disable_downsampling(void);

    void enable_fixed_exposure_gain(void);
    void disable_fixed_exposure_gain(void);

    void set_autogain_exposure_priority(xf val);
    xf   get_autogain_exposure_priority(void);

    void set_autogain_exposure_target_level(xf val);
    xf   get_autogain_exposure_target_level(void);

    void set_autoexposure_top_limit(int tLimit);
    int  get_autoexposure_top_limit(void);

    void set_autogain_top_limit(xf tG);
    xf   get_autogain_top_limit(void);

    void set_total_bandwidth(int b);
    int  get_total_bandwidth(void);
    void set_bandwidth_margin(int m);
    int  get_bandwidth_margin(void);
    xf   get_max_frame_rate(void);

    void set_transfer_format(TransferFormat_t tf);
    TransferFormat_t get_transfer_format(void);

    int  get_exposure(void);
    xf   get_gain(void);
    void put_WB_coefficients(xf& r, xf& g, xf& b);

    void enable_external_trigger(int nextImageTimeout_ms);
    void disable_external_trigger(void);
    bool is_external_triger(void);
    int  get_next_image_timeout(void);

    void enable_external_timestamp_reset(void);
    void disable_external_timestamp_reset(void);
    bool is_external_timestamp_reset(void);

    void set_self_adjust_trail_loops(int t);
    int  get_self_adjust_trail_loops(void);

    void set_custom_AEAG(AEAG* aeag);
    void set_custom_AEAG_target_brightness_level(int level);
    int  get_custom_AEGA_target_brightness_level(void);
    void enable_custom_AEAG(void);
    void disable_custom_AEAG(void);
    bool is_custom_AEAG_enabled(void);

    void set_image_parameter_evaluator(AEAG* ipe);

    /** Save the mean brightness into the array referred by mb.
     * 
     */
    void put_mean_brightness(int* mb);

    void enable_force_xi_auto_white_balance(void);
    void disable_force_xi_auto_white_balance(void);

    void enable_fixed_white_balance(xf cr, xf cg, xf cb);
    void disable_fixed_white_balance(void);

    void enable_debug(void);
    void disable_debug(void);

protected:
    void prepare_before_opening(void);
    void open_and_common_settings(void);
    void set_transfer_format_single_camera(xiAPIplusCameraOcv& cam, TransferFormat_t tf);
    void setup_camera_common(xiAPIplusCameraOcv& cam);
    void set_stereo_external_trigger(void);
    void set_stereo_master_trigger(void);
    void set_stereo_software_trigger(void);
    void set_external_timestamp_reset(void);

    cv::Mat get_single_image(int idx);
    /**
     * @return 0 if no error. 
     */
    int get_images(cv::Mat &img0, cv::Mat &img1);

    /** Call the multi-thread version of the get_single_image.
     * 
     */
    int get_images_mt(cv::Mat &img0, cv::Mat &img1, CameraParams_t &cp0, CameraParams_t &cp1, int n0=1, int n1=1);

    void raw_balance(cv::Mat &img);

    void put_single_camera_params(xiAPIplusCameraOcv &cam, CameraParams_t &cp);

    int EXPOSURE_MILLISEC(int val);
    int EXPOSURE_FROM_MICROSEC(int val);

    void record_settings(int nFrames, std::vector<CameraParams_t> &cp, bool verbose = false);
    void self_adjust_exposure_gain(std::vector<CameraParams_t> &cp);
    void self_adjust_white_balance(std::vector<CameraParams_t> &cp);
    void set_exposure_gain(int idx, int e, xf g);
    void set_white_balance(int idx, xf r, xf g, xf b);
    void set_white_balance(xiAPIplusCameraOcv& cam, xf r, xf g, xf b);
    void apply_custom_AEAG(cv::Mat &img0, cv::Mat &img1, CameraParams_t &camP0, CameraParams_t &camP1);
    void evaluate_image_parameters(cv::Mat &img0, cv::Mat &img1, CameraParams_t &camP0, CameraParams_t &camP1);
    
public:
    const xf  AUTO_GAIN_EXPOSURE_PRIORITY_MAX;
    const xf  AUTO_GAIN_EXPOSURE_PRIORITY_MIM;
    const xf  AUTO_GAIN_EXPOSURE_PRIORITY_DEFAULT;
    const xf  AUTO_GAIN_EXPOSURE_TARGET_LEVEL_MAX;
    const xf  AUTO_GAIN_EXPOSURE_TARGET_LEVEL_MIN;
    const xf  AUTO_GAIN_EXPOSURE_TARGET_LEVEL_DEFAULT;
    const int AUTO_EXPOSURE_TOP_LIMIT_MAX;         // Microsecond.
    const int AUTO_EXPOSURE_TOP_LIMIT_MIN;         // Microsecond.
    const int AUTO_EXPOSURE_TOP_LIMIT_DEFAULT;     // Microsecond.
    const xf  AUTO_GAIN_TOP_LIMIT_MAX;             // db.
    const xf  AUTO_GAIN_TOP_LIMIT_MIN;             // db.
    const xf  AUTO_GAIN_TOP_LIMIT_DEFAULT;         // db.
    const int TOTAL_BANDWIDTH_MAX;                 // MBit/s.
    const int TOTAL_BANDWIDTH_MIN;                 // MBit/s.
    const int BANDWIDTH_MARGIN_MAX;                // %.
    const int BANDWIDTH_MARGIN_MIN;                // %.
    const int BANDWIDTH_MARGIN_DEFAULT;            // %.

protected:
    const int TRIGGER_SOFTWARE;
    const int EXPOSURE_MILLISEC_BASE;

    const int CAM_IDX_0;
    const int CAM_IDX_1;

    const int XI_DEFAULT_TOTAL_BANDWIDTH;
    const int XI_DEFAULT_BANDWIDTH_MARGIN;

    std::string mCamSN[N_XI_C];

    xiAPIplusCameraOcv mCams[N_XI_C];

    cv::Mat mGrayMatBuffer[N_XI_C];

    int mHeight;
    int mWidth;

    int mXi_DownsamplingType; // 1 - binning, 2 - skipping. Only 2 has effect.
    int mXi_Downsampling;     // 1 - Fullsize, 2 - 2x2. Only 2 has effect.

    bool mFixedXiExposureGain;
    xf  mXi_AutoGainExposurePriority;
    xf  mXi_AutoGainExposureTargetLevel;
    int mXi_AutoExposureTopLimit;     // Microsecond.
    int mXi_AutoGainTopLimit;         // db.
    int mXi_TotalBandwidth;           // MBit/s.
    int mXi_BandwidthMargin;          // %.
    xf  mXi_MaxFrameRate;             // fps.

    TransferFormat_t mTransferFormat;

    bool mIsExternalTriggered;
    int mXi_NextImageTimeout_ms;      // Millisecond.

    bool mIsExternalTimeStampReset;

    int mSelfAdjustNumOmittedFrames;
    int mSelfAdjustNumFrames;
    int mSelfAdjustNumTrialLoops;     // The maximum number of trial loops for the recording process of self-adjust operation.
    bool mIsSelfAdjusting;

    int mXi_Exposure; // Microsecond.
    xf  mXi_Gain;

    // Custom auto-exposure-auto-gain (AEAG).
    AEAG* mCAEAG;
    int  mCAEAG_TargetBrightnessLevel;
    int  mCAEAG_TargetBrightnessLevel8Bit;
    bool mCAEAG_IsEnabled;
    int  mMeanBrightness[2];

    bool    mRawBalance;
    cv::Mat mRawBalanceBR;
    cv::Mat mRawBalanceBuffer;

    bool mForceXiAutoWhiteBalance;

    bool mFixedWB;
    xf   mWB_R;
    xf   mWB_G;
    xf   mWB_B;

    // Image parameter evaluator.
    AEAG* mIPE;

    // Debug flag.
    bool mFlagDebug;
};

}

#endif /* __STEREOXICAMERA_H__ */