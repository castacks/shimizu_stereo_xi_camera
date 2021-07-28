
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

#include <chrono>
#include <iostream>
#include <thread>

// ========= Includes for ROS and OpenCV. ===================

#include "ROSNode/SXCSync.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ros/ros.h"

#include "Profiler/Profiler.hpp"

using namespace cv;

// ============ Static global variables. ==========

const char* NODE_NAME = "xic_stereo";

// =============== main(). =========================

int main(int argc, char* argv[])
{
	// Return value.
	int ret = 0;
	SRN::Res_t nodeRes    = SRN::RES_OK;
	SRN::ProcessType_t pt = SRN::PROCESS_CONTINUE;

	// Create SyncROSNode object.
	SRN::SXCSync sxcSync(NODE_NAME);

	// Initialization ROS.
	nodeRes = sxcSync.init(argc, argv, NODE_NAME); CHECK_RES(nodeRes);

	// Handle launch parameter.
	nodeRes = sxcSync.parse_launch_parameters(); CHECK_RES(nodeRes);

	// Synchronization preparation.
	nodeRes = sxcSync.prepare(); CHECK_RES(nodeRes);

	while ( ros::ok() && true == sxcSync.is_looping() )
	{
		// Resume.
		pt = SRN::PROCESS_CONTINUE;
		while ( ros::ok() && SRN::PROCESS_CONTINUE == pt )
		{
			nodeRes = sxcSync.resume(pt); CHECK_RES(nodeRes);
		}

		// Begin synchronizing ROS node.
		pt = SRN::PROCESS_CONTINUE;
		while( ros::ok() && SRN::PROCESS_CONTINUE == pt )
		{
			nodeRes = sxcSync.synchronize(pt); CHECK_RES(nodeRes);
		}

		// Pause.
		pt = SRN::PROCESS_CONTINUE;
		while ( ros::ok() && SRN::PROCESS_CONTINUE == pt )
		{
			nodeRes = sxcSync.pause(pt); CHECK_RES(nodeRes);
		}
	}

	// Stop.
	nodeRes = sxcSync.stop(); CHECK_RES(nodeRes);
	
	// cvWaitKey(500); // OpenCV 4.5 has trouble with this.
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	// Destroy.
	nodeRes = sxcSync.destroy(); CHECK_RES(nodeRes);

	PROFILER_SAVE("Profiler.csv", gProfilers);

	return ret;
}
