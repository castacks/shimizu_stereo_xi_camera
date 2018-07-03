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

// ============ File-wise or global variables. ===========

/* No variables declared or initialized. */

// ============= Class definitions. ======================

namespace sxc {

struct exception_base        : virtual std::exception, virtual boost::exception { };
struct bad_argument          : virtual exception_base { };
struct argument_out_of_range : virtual bad_argument { };
struct argument_null         : virtual bad_argument { };
struct camera_api_exception  : virtual exception_base { };

typedef boost::error_info<struct tag_info_string, std::string> ExceptionInfoString;

class StereoXiCamera 
{
public:
    typedef struct CameraParams
    {
        int AEAGEnabled;  // 1 for enabled.
        xf  AEAGPriority;
        int exposure;     // Milliseconds.
        xf  gain;         // db.
    } CameraParams_t;

public:
    StereoXiCamera(std::string &camSN0, std::string &camSN1);
    ~StereoXiCamera();

    void open();
    void self_adjust(bool verbose = false);
    void start_acquisition(int waitMS = 500);

    void software_trigger(void);
    void get_images(cv::Mat &img0, cv::Mat &img1, CameraParams_t &camP0, CameraParams_t &camP1);

    void stop_acquisition(int waitMS = 500);
    void close();

    void put_sensor_filter_array(int idx, std::string &strFilterArray);

    // Getters and setters.
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

    int  get_exposure(void);
    xf   get_gain(void);

    void set_custom_AEAG(AEAG* aeag);
    void enable_custom_AEAG(void);
    void disable_custom_AEAG(void);
    bool is_custom_AEAG_enabled(void);

protected:
    void prepare_before_opening();
    void open_and_common_settings();
    void setup_camera_common(xiAPIplusCameraOcv& cam);

    void get_images(cv::Mat &img0, cv::Mat &img1);
    cv::Mat get_single_image(int idx);
    void put_single_camera_params(xiAPIplusCameraOcv &cam, CameraParams_t &cp);

    int EXPOSURE_MILLISEC(int val);

    void record_settings(int nFrames, std::vector<CameraParams_t> &cp, bool verbose = false);
    void self_adjust_exposure_gain(std::vector<CameraParams_t> &cp);
    void self_adjust_white_balance(std::vector<CameraParams_t> &cp);
    void set_exposure_gain(int idx, int e, xf g);
    void apply_custom_AEAG(cv::Mat &img0, cv::Mat &img1, CameraParams_t &camP0, CameraParams_t &camP1);
    
public:
    const xf  AUTO_GAIN_EXPOSURE_PRIORITY_MAX;
    const xf  AUTO_GAIN_EXPOSURE_PRIORITY_MIM;
    const xf  AUTO_GAIN_EXPOSURE_PRIORITY_DEFAULT;
    const xf  AUTO_GAIN_EXPOSURE_TARGET_LEVEL_MAX;
    const xf  AUTO_GAIN_EXPOSURE_TARGET_LEVEL_MIN;
    const xf  AUTO_GAIN_EXPOSURE_TARGET_LEVEL_DEFAULT;
    const int AUTO_EXPOSURE_TOP_LIMIT_MAX;         // Millisecond.
    const int AUTO_EXPOSURE_TOP_LIMIT_MIN;         // Millisecond.
    const int AUTO_EXPOSURE_TOP_LIMIT_DEFAULT;     // Millisecond.
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

    xf  mXi_AutoGainExposurePriority;
    xf  mXi_AutoGainExposureTargetLevel;
    int mXi_AutoExposureTopLimit;     // Milisecond.
    int mXi_AutoGainTopLimit;         // db.
    int mXi_TotalBandwidth;           // MBit/s.
    int mXi_BandwidthMargin;          // %.
    xf  mXi_MaxFrameRate;             // fps.

    int mSelfAdjustNumOmittedFrames;
    int mSelfAdjustNumFrames;
    bool mIsSelfAdjusting;

    int mXi_Exposure; // Milisecond.
    xf  mXi_Gain;

    // Custom auto-exposure-auto-gain (AEAG).
    AEAG* mCAEAG;
    bool mCAEAG_IsEnabled;
};

}

#endif /* __STEREOXICAMERA_H__ */