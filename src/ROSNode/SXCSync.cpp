#include "ROSNode/SXCSync.hpp"

using namespace cv;
using namespace SRN;

SXCSync::SXCSync(const std::string& name)
: SyncROSNode(name),
  CAM_0_IDX(0), CAM_1_IDX(1),
  mTopicNameLeftImage("left/image_raw"), mTopicNameRightImage("right/image_raw"), 
  mOutDir("./")
{
    mXiCameraSN[0] = "CUCAU1814018";
    mXiCameraSN[1] = "CUCAU1814020";
}

SXCSync::~SXCSync()
{

}

int SXCSync::prepare(void)
{
    return 0;
}

int SXCSync::synchronize(void)
{
    return 0;
}

int SXCSync::pause(void)
{
    return 0;
}

int SXCSync::destroy(void)
{
    return 0;
}

void SXCSync::set_topic_name_left_image(const std::string& name)
{
    mTopicNameLeftImage = name;
}

const std::string& SXCSync::get_topi_name_left_image(void)
{
    return mTopicNameLeftImage;
}

void SXCSync::set_topic_name_right_image(const std::string& name)
{
    mTopicNameRightImage = name;
}

const std::string& SXCSync::get_topi_name_right_image(void)
{
    return mTopicNameRightImage;
}

void SXCSync::set_out_dir(const std::string& outDir)
{
    mOutDir = outDir;
}

const std::string& SXCSync::get_out_dir(void)
{
    return mOutDir;
}

void SXCSync::parse_launch_parameters(void)
{
    
}