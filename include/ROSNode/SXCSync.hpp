#ifndef __SXC_SYNC_HPP__
#define __SXC_SYNC_HPP__

#include "ROSNode/SyncROSNode.hpp"

namespace SRN
{

class SXCSync : public SyncROSNode
{
public:
    SXCSync();
    virtual ~SXCSync();

    int prepare(void);
    int synchronize(void);
    int pause(void);
    int destroy(void);
};

}

#endif // __SXC_SYNC_HPP__