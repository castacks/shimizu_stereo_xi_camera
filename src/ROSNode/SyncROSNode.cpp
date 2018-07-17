
#include <iostream>

#include "ROSNode/SyncROSNode.hpp"

using namespace SRN;

int SyncROSNode::NODE_COUNT = 0;

SyncROSNode::SyncROSNode(const std::string& name)
: mpROSNode(NULL), mNodeName(name)
{

}

SyncROSNode::~SyncROSNode()
{
    if ( NULL != mpROSNode )
    {
        delete mpROSNode; mpROSNode = NULL;
    }
}

int SyncROSNode::init(int& argc, char** argv, std::string& name, uint32_t options)
{
    if ( 0 == NODE_COUNT )
    {
        ros::init(argc, argv, name, options);
        mpROSNode = new ros::NodeHandle("~");

        NODE_COUNT++;

        return 0;
    }
    else
    {
        std::cout << "Error: Only one init() is allowed." << std::endl;
        return -1;
    }
}

int SyncROSNode::parse_launch_parameters(void)
{
    return 0;
}

int SyncROSNode::prepare(void)
{
    return 0;
}

int SyncROSNode::synchronize(void)
{
    return 0;
}

int SyncROSNode::pause(void)
{
    return 0;
}

int SyncROSNode::destroy(void)
{
    return 0;
}