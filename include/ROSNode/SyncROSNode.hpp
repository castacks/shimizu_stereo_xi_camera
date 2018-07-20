#ifndef __SYNC_ROS_NODE_HPP__
#define __SYNC_ROS_NODE_HPP__

#include <string>

#include "ros/ros.h"

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

namespace SRN
{

class SyncROSNode
{
public:
    SyncROSNode(const std::string& name);
    virtual ~SyncROSNode();

    int init(int& argc, char** argv, std::string& name, uint32_t options = 0);
	virtual int parse_launch_parameters(void);
    virtual int prepare(void);
	virtual int resume(void);
    virtual int synchronize(void);
    virtual int pause(void);
    virtual int destroy(void);

protected:
    ros::NodeHandle* mpROSNode;
    std::string mNodeName;

private:
    static int NODE_COUNT;
};

}

#endif // __SYNC_ROS_NODE_HPP__