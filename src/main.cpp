
/**
 * Test code for the stereo camera built on the xiC models produced by XIMEA.
 *
 * Author
 * ======
 *
 * Yaoyu Hu <yyhu_live@outlook.com>
 *
 * Date
 * ====
 *
 * Created:  2018-06-04
 * Modified: 2018-06-19
 *
 */

#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <stdio.h>

#include "AEAG/MeanBrightness.hpp"
#include "StereoXiCamera.hpp"

// ========= Includes for ROS and OpenCV. ===================

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

// ============ Static global variables. ==========

const char* NODE_NAME = "xic_stereo";
const char* TOPIC_NAME_LEFT_IMAGE  = "left/image_raw";
const char* TOPIC_NAME_RIGHT_IMAGE = "right/image_raw";

const int N_IMAGES = 0;

const std::string OUT_DIR = "/home/yyhu/ROS/p2/catkin/src/stereo_xi_camera/output";

std::string XI_CAMERA_SN_1 = "CUCAU1814018";
std::string XI_CAMERA_SN_0 = "CUCAU1814020";

const double DEFAULT_AUTO_GAIN_EXPOSURE_PRIORITY     = 0.9;
const double DEFAULT_AUTO_GAIN_EXPOSURE_TARGET_LEVEL = 40.0;
const int    DEFAULT_AUTO_EXPOSURE_TOP_LIMIT         = 200000;  // Microsecond.
const int    DEFAULT_AUTO_GAIN_TOP_LIMIT             = 12;   // dB.
const int    DEFAULT_TOTAL_BANDWIDTH                 = 2400;
const int    DEFAULT_BANDWIDTH_MARGIN                = 10;
const int    DEFAULT_LOOP_RATE                       = 3;

const int    DEFAULT_NEXT_IMAGE_TIMEOUT_MS           = 1000;

const double DEFAULT_CUSTOM_AEAG_PRIORITY           = 0.9;
const double DEFAULT_CUSTOM_AEAG_EXPOSURE_TOP_LIMIT = 200000.0; // Mirosecond.
const double DEFAULT_CUSTOM_AEAG_GAIN_TOP_LIMIT     = 12.0;  // dB.
const int    DEFAULT_CUSTOM_AEAG_BRIGHTNESS_LEVEL   = 30;    // %.

// ============= Local macros. =====================

#define ROSLAUNCH_GET_PARAM(nh, name, var, d) \
	{\
		std::stringstream var##_ss;\
		\
		if ( false == nh.getParam(name, var) ) \
		{\
			var = d;\
			var##_ss << d;\
			ROS_INFO("Parameter %s is not present. Use default value %s.", name, var##_ss.str().c_str());\
		}\
		else\
		{\
			var##_ss << var;\
			ROS_INFO("Parameter %s = %s.", name, var##_ss.str().c_str());\
		}\
	}

// =============== main(). =========================

int main(int argc, char* argv[])
{
	int ret = 0;

	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle nodeHandle("~");

	// Get the parameters.
	double pAutoGainExposurePriority    = DEFAULT_AUTO_GAIN_EXPOSURE_PRIORITY;
	double pAutoGainExposureTargetLevel = DEFAULT_AUTO_GAIN_EXPOSURE_TARGET_LEVEL;
	int    pAutoExposureTopLimit        = DEFAULT_AUTO_EXPOSURE_TOP_LIMIT; // Microsecond.
	int    pAutoGainTopLimit            = DEFAULT_AUTO_GAIN_TOP_LIMIT;
	int    pTotalBandwidth              = DEFAULT_TOTAL_BANDWIDTH;
	int    pBandwidthMargin             = DEFAULT_BANDWIDTH_MARGIN;
	int    pFlagWriteImage              = 0;
	int    pLoopRate                    = DEFAULT_LOOP_RATE;
	std::string pOutDir                 = OUT_DIR;

	int    pExternalTrigger             = 0;
	int    pNextImageTimeout_ms         = DEFAULT_NEXT_IMAGE_TIMEOUT_MS;

	int    pCustomAEAGEnabled          = 0;
	double pCustomAEAGPriority         = DEFAULT_CUSTOM_AEAG_PRIORITY;
	double pCustomAEAGExposureTopLimit = DEFAULT_CUSTOM_AEAG_EXPOSURE_TOP_LIMIT;
	double pCustomAEAGGainTopLimit     = DEFAULT_CUSTOM_AEAG_GAIN_TOP_LIMIT;
	int    pCustomAEAGBrightnessLevel  = DEFAULT_CUSTOM_AEAG_BRIGHTNESS_LEVEL;

	std::string pXICameraSN_0 = XI_CAMERA_SN_0;
	std::string pXICameraSN_1 = XI_CAMERA_SN_1;

	ROSLAUNCH_GET_PARAM(nodeHandle, "pAutoGainExposurePriority", pAutoGainExposurePriority, DEFAULT_AUTO_GAIN_EXPOSURE_PRIORITY);
	ROSLAUNCH_GET_PARAM(nodeHandle, "pAutoGainExposureTargetLevel", pAutoGainExposureTargetLevel, DEFAULT_AUTO_GAIN_EXPOSURE_TARGET_LEVEL);
	ROSLAUNCH_GET_PARAM(nodeHandle, "pAutoExposureTopLimit", pAutoExposureTopLimit, DEFAULT_AUTO_EXPOSURE_TOP_LIMIT);
	ROSLAUNCH_GET_PARAM(nodeHandle, "pAutoGainTopLimit", pAutoGainTopLimit, DEFAULT_AUTO_GAIN_TOP_LIMIT);
	ROSLAUNCH_GET_PARAM(nodeHandle, "pTotalBandwidth", pTotalBandwidth, DEFAULT_TOTAL_BANDWIDTH);
	ROSLAUNCH_GET_PARAM(nodeHandle, "pBandwidthMargin", pBandwidthMargin, DEFAULT_BANDWIDTH_MARGIN);
	ROSLAUNCH_GET_PARAM(nodeHandle, "pLoopRate", pLoopRate, DEFAULT_LOOP_RATE);
	ROSLAUNCH_GET_PARAM(nodeHandle, "pFlagWriteImage", pFlagWriteImage, 0);
	ROSLAUNCH_GET_PARAM(nodeHandle, "pOutDir", pOutDir, OUT_DIR);

	ROSLAUNCH_GET_PARAM(nodeHandle, "pExternalTrigger", pExternalTrigger, 0);
	ROSLAUNCH_GET_PARAM(nodeHandle, "pNextImageTimeout_ms", pNextImageTimeout_ms, DEFAULT_NEXT_IMAGE_TIMEOUT_MS);

	ROSLAUNCH_GET_PARAM(nodeHandle, "pCustomAEAGEnabled", pCustomAEAGEnabled, 0);
	ROSLAUNCH_GET_PARAM(nodeHandle, "pCustomAEAGPriority", pCustomAEAGPriority, DEFAULT_CUSTOM_AEAG_PRIORITY);
	ROSLAUNCH_GET_PARAM(nodeHandle, "pCustomAEAGExposureTopLimit", pCustomAEAGExposureTopLimit, DEFAULT_CUSTOM_AEAG_EXPOSURE_TOP_LIMIT);
	ROSLAUNCH_GET_PARAM(nodeHandle, "pCustomAEAGGainTopLimit", pCustomAEAGGainTopLimit, DEFAULT_CUSTOM_AEAG_GAIN_TOP_LIMIT);
	ROSLAUNCH_GET_PARAM(nodeHandle, "pCustomAEAGBrightnessLevel", pCustomAEAGBrightnessLevel, DEFAULT_CUSTOM_AEAG_BRIGHTNESS_LEVEL);

	ROSLAUNCH_GET_PARAM(nodeHandle, "pXICameraSN_0", pXICameraSN_0, XI_CAMERA_SN_0);
	ROSLAUNCH_GET_PARAM(nodeHandle, "pXICameraSN_1", pXICameraSN_1, XI_CAMERA_SN_1);

	image_transport::ImageTransport imageTransport(nodeHandle);

	image_transport::Publisher publishersImage[2] = { 
		imageTransport.advertise(TOPIC_NAME_LEFT_IMAGE, 1), 
		imageTransport.advertise(TOPIC_NAME_RIGHT_IMAGE, 1) };

  	ros::Rate loop_rate(pLoopRate);

	// The ROS message to be published.
	sensor_msgs::ImagePtr msgImage;

	// The object of stereo camera based on the XIMEA cameras.
	sxc::StereoXiCamera stereoXiCamera = sxc::StereoXiCamera(pXICameraSN_0, pXICameraSN_1);

	// Trigger selection.
	if ( 1 == pExternalTrigger )
	{
		stereoXiCamera.enable_external_trigger(pNextImageTimeout_ms);
	}

	// The custom AEAG object.
	sxc::MeanBrightness* mbAEAG = NULL;

	if ( 1 == pCustomAEAGEnabled )
	{
		mbAEAG = new sxc::MeanBrightness;
		mbAEAG->set_exposure_top_limit(pCustomAEAGExposureTopLimit * 1000);
		mbAEAG->set_gain_top_limit(sxc::dBToGain(pCustomAEAGGainTopLimit));
		mbAEAG->set_priority(pCustomAEAGPriority);

		stereoXiCamera.set_custom_AEAG(mbAEAG);
		stereoXiCamera.set_custom_AEAG_target_brightness_level(pCustomAEAGBrightnessLevel);
		stereoXiCamera.enable_custom_AEAG();
	}

	// Run the ROS node.
	try
	{
		// Configure the stereo camera.
		stereoXiCamera.set_autogain_exposure_priority(pAutoGainExposurePriority);
		stereoXiCamera.set_autogain_exposure_target_level(pAutoGainExposureTargetLevel);
		stereoXiCamera.set_autoexposure_top_limit(pAutoExposureTopLimit);
		stereoXiCamera.set_autogain_top_limit(pAutoGainTopLimit);
		stereoXiCamera.set_total_bandwidth(pTotalBandwidth);
		stereoXiCamera.set_bandwidth_margin(pBandwidthMargin);

		// Pre-open, open and configure the stereo camera.
		stereoXiCamera.open();

		// Self-adjust.
		ROS_INFO("Perform self-adjust...");
		stereoXiCamera.self_adjust(true);
		ROS_INFO("Self-adjust done.");

		// Get the sensor array.
		std::string strSensorArray;
		stereoXiCamera.put_sensor_filter_array(0, strSensorArray);
		ROS_INFO("The sensor array string is %s.", strSensorArray.c_str());

		// Start acquisition.
		ROS_INFO("%s", "Start acquisition.");
		stereoXiCamera.start_acquisition();

		// Temporary variables.
		Mat cvImages[2];        // OpenCV Mat array to hold the images.
		sxc::StereoXiCamera::CameraParams_t cp[2]; // Camera parameters.
		std::stringstream ss;   // String stream for outputing info.
		ros::Time rosTimeStamp; // ROS time stamp for the header of published ROS image messages.

		std::vector<int> jpegParams;
		jpegParams.push_back( CV_IMWRITE_JPEG_QUALITY );
		jpegParams.push_back( 100 );

		int nImages = 0; // Image counter.

		// Begin running ROS node.
		while(ros::ok())
		{
			if ( 0 != N_IMAGES )
			{
				if ( nImages >= N_IMAGES )
				{
					// Stop this ROS node.
					break;
				}
			}

			ROS_INFO("nImages = %d", nImages);

			// Trigger.
			if ( false == stereoXiCamera.is_external_triger() )
			{
				stereoXiCamera.software_trigger();
			}

			// Get images.
			stereoXiCamera.get_images( cvImages[0], cvImages[1], cp[0], cp[1] );

			// Prepare the time stamp for the header.
			rosTimeStamp = ros::Time::now();

			// Convert the image into ROS image message.
			LOOP_CAMERAS_BEGIN
				// Clear the temporary sting stream.
				ss.flush();	ss.str(""); ss.clear();
				ss << pOutDir << "/" << nImages << "_" << loopIdx;

				ROS_INFO( "%s", ss.str().c_str() );

				// Save the captured image to file system.
				if ( 1 == pFlagWriteImage )
				{
					std::string yamlFilename = ss.str() + ".yaml";
					std::string imgFilename = ss.str() + ".bmp";

					// FileStorage cfFS(yamlFilename, FileStorage::WRITE);
					// cfFS << "frame" << nImages << "image_id" << loopIdx << "raw_data" << cvImages[loopIdx];
					imwrite(imgFilename, cvImages[loopIdx], jpegParams);
				}
				
				ROS_INFO( "Camera %d captured image (%d, %d). AEAG %d, AEAGP %.2f, exp %.3f, gain %.1f dB.", 
				          loopIdx, cvImages[loopIdx].rows, cvImages[loopIdx].cols,
						  cp[loopIdx].AEAGEnabled, cp[loopIdx].AEAGPriority, cp[loopIdx].exposure / 1000.0, cp[loopIdx].gain );

				// Publish images.
				msgImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cvImages[loopIdx]).toImageMsg();
				// msgImage = cv_bridge::CvImage(std_msgs::Header(), "bayer_bggr8", cvImages[loopIdx]).toImageMsg();
				msgImage->header.seq   = nImages;
				msgImage->header.stamp = rosTimeStamp;

				publishersImage[loopIdx].publish(msgImage);

				ROS_INFO("%s", "Message published.");
			LOOP_CAMERAS_END

			// ROS spin.
			ros::spinOnce();

			// Sleep().
			loop_rate.sleep();

			++nImages;
		}

		// Stop acquisition.
		stereoXiCamera.stop_acquisition();

		cvWaitKey(500);

		ROS_INFO("Stereo camera stopped.");

		// Close.
		stereoXiCamera.close();
		ROS_INFO("Stereo camera closed.");

		// Delete AEAG object.
		if ( NULL != mbAEAG )
		{
			delete mbAEAG; mbAEAG = NULL;
		}
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

		if ( NULL != mbAEAG )
		{
			delete mbAEAG; mbAEAG = NULL;
		}

		ret = -1;
	}

	return ret;
}
