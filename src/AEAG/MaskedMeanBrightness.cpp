#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

#include "AEAG/MaskedMeanBrightness.hpp"

using namespace sxc;

MaskedMeanBrightness::MaskedMeanBrightness(const std::string& maskFn)
: MeanBrightness(),
  mMaskN(0), mMaskX(NULL), mMaskY(NULL), mMaskF(NULL)
{
    if ( 0 == maskFn.compare("") )
    {
        throw std::runtime_error("maskFn is empty string.");
    }

    read_exposure_mask(maskFn);
}

MaskedMeanBrightness::~MaskedMeanBrightness()
{
    destroy();
}

void MaskedMeanBrightness::destroy(void)
{
    if ( NULL != mMaskF )
    {
        delete [] mMaskF; mMaskF = NULL;
    }

    if ( NULL != mMaskY )
    {
        delete [] mMaskY; mMaskY = NULL;
    }

    if ( NULL != mMaskX )
    {
        delete [] mMaskX;
    }
}

void MaskedMeanBrightness::read_exposure_mask(const std::string& fn)
{
    std::ifstream ifs(fn.c_str());

    if ( !ifs.good() )
    {
        std::stringstream ss;
        ss << "Could not open " << fn;
        throw std::runtime_error(ss.str());
    }

    std::vector<int> x, y;
    std::vector<xf> f;

    int tempX, tempY;
    xf tempF;

    while ( !ifs.eof() )
    {
        ifs >> tempX >> tempY >> tempF;
        x.push_back(tempX);
        y.push_back(tempY);
        f.push_back(tempF);
    }

    std::cout << "MaskedMeanBrightness mask read." << std::endl;
    std::cout << f.size() << " entries read." << std::endl;

    destroy();

    mMaskX = new int[f.size()];
    mMaskY = new int[f.size()];
    mMaskF = new xf[f.size()];

    for (int i = 0; i < f.size(); ++i)
    {
        mMaskX[i] = x.at(i);
        mMaskY[i] = y.at(i);
        mMaskF[i] = f.at(i);
    }

    mMaskN = f.size();

    ifs.close();
}

xf MaskedMeanBrightness::get_mean_brightness(cv::InputArray _img)
{
    std::cout << "In MaskedMeanBrightness::get_mean_brightness()" << std::endl;
    
    cv::Mat img = _img.getMat();

    int dataType = img.type();

    if ( dataType != CV_8U && dataType != CV_8UC1 )
    {
        std::cout << "get_mean_brightness(): dataType = " << dataType << std::endl;
        return -1;
    }

    xf meanBrightness = 0.0f;

    for ( int i = 0; i < mMaskN; ++i )
    {
        meanBrightness += img.at<uint8_t>( mMaskY[i], mMaskX[i] ) * mMaskF[i];
    }

    return meanBrightness;
}
