#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

#include "AEAG/DownSampledMeanBrightness.hpp"

#define RELEASE_ARRAY(x) \
    if ( NULL != x ) \
    {\
        delete [] x; x = NULL; \
    }

using namespace sxc;

DownSampledMeanBrightness::DownSampledMeanBrightness(int nx, int ny, int blockSamplesX, int blockSamplesY)
: MeanBrightness(),
  CR(0.299), CG(0.587/2), CB(0.114),
  mBSX(0), mBSY(0),
  mX(NULL), mY(NULL), mC(NULL), mN(0)
{
    fill_indices(nx, ny, blockSamplesX, blockSamplesY);
}

DownSampledMeanBrightness::~DownSampledMeanBrightness()
{
    destroy();
}

void DownSampledMeanBrightness::destroy(void)
{
    RELEASE_ARRAY(mC);
    RELEASE_ARRAY(mY);
    RELEASE_ARRAY(mX);

    mN = 0;
}

void DownSampledMeanBrightness::fill_indices(int nx, int ny, int blockSamplesX, int blockSamplesY)
{
    const int maxBlocksX = nx / 2;
    const int maxBlocksY = ny / 2;

    blockSamplesX = blockSamplesX > maxBlocksX ? maxBlocksX : blockSamplesX;
    blockSamplesY = blockSamplesY > maxBlocksY ? maxBlocksY : blockSamplesY;

    const int strideX = ( nx - 2 ) / ( blockSamplesX - 1 );
    const int strideY = ( ny - 2 ) / ( blockSamplesY - 1 );

    // Allocate memory.
    destroy();

    mN = blockSamplesX*2 * blockSamplesY*2;

    mX = new int[mN];
    mY = new int[mN];
    mC = new xf[mN];

    mBSX = blockSamplesX;
    mBSY = blockSamplesY;

    int idx = 0;

    for ( int i = 0; i < ny; i += strideY )
    {
        for ( int j = 0; j < nx; j += strideX )
        {
            // BGGR pattern.
            mX[idx] = j;
            mY[idx] = i;
            mC[idx] = CB;
            idx++;

            mX[idx] = j+1;
            mY[idx] = i;
            mC[idx] = CG;
            idx++;

            mX[idx] = j;
            mY[idx] = i+1;
            mC[idx] = CG;
            idx++;

            mX[idx] = j+1;
            mY[idx] = i+1;
            mC[idx] = CR;
            idx++;
        }
    }
}

int DownSampledMeanBrightness::get_mean_brightness(cv::InputArray _img)
{
    std::cout << "In DownSampledMeanBrightness::get_mean_brightness()" << std::endl;
    
    cv::Mat img = _img.getMat();

    int dataType = img.type();

    if ( dataType != CV_8U && dataType != CV_8UC1 )
    {
        std::cout << "get_mean_brightness(): dataType = " << dataType << std::endl;
        return -1;
    }

    xf meanBrightness = 0.0f;

    for ( int i = 0; i < mN; ++i )
    {
        meanBrightness += img.at<uint8_t>( mY[i], mX[i] ) * mC[i];
    }

    return static_cast<int>(meanBrightness / ( mN/4 )  );
}
