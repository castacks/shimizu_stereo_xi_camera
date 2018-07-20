
#include <iostream>

#include "ROSNode/SyncROSNode.hpp"

using namespace SRN;

int SyncROSNode::NODE_COUNT = 0;

SyncROSNode::SyncROSNode(const std::string& name)
: mpROSNode(NULL), mNodeName(name), mIsLooping(true)
{

}

SyncROSNode::~SyncROSNode()
{
    if ( NULL != mpROSNode )
    {
        delete mpROSNode; mpROSNode = NULL;
    }
}

Res_t SyncROSNode::init(int& argc, char** argv, const std::string& name, uint32_t options)
{
    if ( 0 == NODE_COUNT )
    {
        ros::init(argc, argv, name, options);
        mpROSNode = new ros::NodeHandle("~");

        NODE_COUNT++;

        return RES_OK;
    }
    else
    {
        std::cout << "Error: Only one init() is allowed." << std::endl;
        return RES_ERROR;
    }
}

Res_t SyncROSNode::parse_launch_parameters(void)
{
    return RES_OK;
}

Res_t SyncROSNode::prepare(void)
{
    return RES_OK;
}

Res_t SyncROSNode::resume(ProcessType_t& pt)
{
    pt = PROCESS_ONCE;
    return RES_OK;
}

Res_t SyncROSNode::synchronize(ProcessType_t& pt)
{
    pt = PROCESS_ONCE;
    return RES_OK;
}

Res_t SyncROSNode::pause(ProcessType_t& pt)
{
    pt = PROCESS_ONCE;
    return RES_OK;
}

Res_t SyncROSNode::stop(void)
{
    return RES_OK;
}

Res_t SyncROSNode::destroy(void)
{
    return RES_OK;
}

bool SyncROSNode::is_looping(void)
{
    return mIsLooping;
}

void SyncROSNode::continue_looping(void)
{
    mIsLooping = true;
}

void SyncROSNode::stop_looping(void)
{
    mIsLooping = false;
}