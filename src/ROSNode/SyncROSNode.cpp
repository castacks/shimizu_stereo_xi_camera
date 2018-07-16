#include "ROSNode/SyncROSNode.hpp"

using namespace SRN;

SyncROSNode::SyncROSNode()
: mpROSNode(NULL)
{

}

SyncROSNode::~SyncROSNode()
{

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