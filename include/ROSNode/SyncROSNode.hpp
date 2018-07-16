#ifndef __SYNC_ROS_NODE_HPP__
#define __SYNC_ROS_NODE_HPP__

#include "ros/ros.h"

namespace SRN
{

class SyncROSNode
{
public:
    SyncROSNode();
    virtual ~SyncROSNode();

    virtual int prepare(void);
    virtual int synchronize(void);
    virtual int pause(void);
    virtual int destroy(void);

protected:
    ros::NodeHandle* mpROSNode;
};

}

#endif // __SYNC_ROS_NODE_HPP__