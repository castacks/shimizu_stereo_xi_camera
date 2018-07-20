#ifndef __SYNC_ROS_NODE_HPP__
#define __SYNC_ROS_NODE_HPP__

#include <sstream>
#include <string>

#include <boost/exception/all.hpp>
#include <boost/shared_ptr.hpp>

#include "ros/ros.h"

// ============= Local macros. =====================

#define CHECK_RES(res) \
    if ( SRN::RES_OK != res )\
	{\
        std::stringstream res##_ss;\
    	res##_ss << "Node returns non-OK value.";\
        BOOST_THROW_EXCEPTION( SRN::res_error() << SRN::ExceptionInfoString(res##_ss.str()) );\
    }

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

struct exception_base : virtual std::exception, virtual boost::exception { };
struct res_error      : virtual exception_base { };

typedef boost::error_info<struct tag_info_string, std::string> ExceptionInfoString;

typedef enum
{
	RES_OK = 0,
	RES_ERROR
} Res_t;

typedef enum
{
	PROCESS_CONTINUE = 0,
	PROCESS_ONCE
} ProcessType_t;

class SyncROSNode
{
public:
    SyncROSNode(const std::string& name);
    virtual ~SyncROSNode();

    Res_t init(int& argc, char** argv, const std::string& name, uint32_t options = 0);
	virtual Res_t parse_launch_parameters(void);
    virtual Res_t prepare(void);
	virtual Res_t resume(ProcessType_t& pt);
    virtual Res_t synchronize(ProcessType_t& pt);
    virtual Res_t pause(ProcessType_t& pt);
	virtual Res_t stop(void);
    virtual Res_t destroy(void);

	bool is_looping(void);

protected:
	void continue_looping(void);
	void stop_looping(void);

protected:
    ros::NodeHandle* mpROSNode;
    std::string mNodeName;
	bool mIsLooping;

private:
    static int NODE_COUNT;
};

}

#endif // __SYNC_ROS_NODE_HPP__