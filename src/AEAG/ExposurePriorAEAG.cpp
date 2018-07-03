#include <iostream>

#include "AEAG/ExposurePriorAEAG.hpp"

using namespace sxc;

ExposurePriorAEAG::ExposurePriorAEAG()
: AEAG(),
  mPriority(0.9)
{

}

ExposurePriorAEAG::~ExposurePriorAEAG()
{

}

void ExposurePriorAEAG::set_priority(xf p)
{
    mPriority = p;
}

xf ExposurePriorAEAG::get_priority(void)
{
    return mPriority;
}
