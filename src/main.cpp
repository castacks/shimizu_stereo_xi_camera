
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

// ========= Includes for ROS and OpenCV. ===================

#include "ROSNode/SXCSync.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ros/ros.h"

using namespace cv;

// ============ Static global variables. ==========

const char* NODE_NAME = "xic_stereo";

// =============== main(). =========================

int main(int argc, char* argv[])
{
	// Return value.
	int ret = 0;
	SRN::Res_t nodeRes = SRN::RES_OK;

	// Create SyncROSNode object.
	SRN::SXCSync sxcSync(NODE_NAME);

	// Initialization ROS.
	nodeRes = sxcSync.init(argc, argv, NODE_NAME); CHECK_RES(nodeRes);

	// Handle launch parameter.
	nodeRes = sxcSync.parse_launch_parameters(); CHECK_RES(nodeRes);

	// Synchronization preparation.
	nodeRes = sxcSync.prepare(); CHECK_RES(nodeRes);

	// Resume.
	nodeRes = sxcSync.resume(); CHECK_RES(nodeRes);

	// Begin synchronizing ROS node.
	while(ros::ok())
	{
		nodeRes = sxcSync.synchronize(); CHECK_RES(nodeRes);
	}

	// Stop.
	nodeRes = sxcSync.pause(); CHECK_RES(nodeRes);
	cvWaitKey(500);

	// Destroy.
	nodeRes = sxcSync.destroy(); CHECK_RES(nodeRes);

	return ret;
}
