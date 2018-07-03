#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "AEAG/AEAG.hpp"

namespace sxc
{

class ExposurePriorAEAG : public AEAG
{
public:
    ExposurePriorAEAG();
    virtual ~ExposurePriorAEAG();

    void set_priority(xf p);
    xf   get_priority(void);

protected:
    xf mPriority;
};


} // namespace sxc
