#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

#include "AEAG/CentralMeanBrightness.hpp"

using namespace sxc;

// The values of CR, CG, and, CB are obtained from
// https://docs.opencv.org/3.4.0/de/d25/imgproc_color_conversions.html
// CG is further divided by 2.

CentralMeanBrightness::CentralMeanBrightness(
    int width, int height,
    xf fX, xf fY, int blockSamplesX, int blockSamplesY, 
    xf fR, xf fG, xf fB)
: MeanBrightness(),
  CR(0.299*fR), CG(0.587/2*fG), CB(0.114*fB),
  mBSX(0), mBSY(0), mN(0)
{
    fill_indices(width, height, fX, fY, blockSamplesX, blockSamplesY);
}

CentralMeanBrightness::~CentralMeanBrightness()
{
    mN = 0;
}

void CentralMeanBrightness::fill_indices(
    int W, int H, xf fX, xf fY, int blockSamplesX, int blockSamplesY)
{
    assert(fX > 0 && fX <= 1);
    assert(fY > 0 && fY <= 1);

    // Figure out the central region.

    const int nx = ( static_cast<int>( W * fX ) / 2 ) * 2;
    const int ny = ( static_cast<int>( H * fY ) / 2 ) * 2;

    const int maxBlocksX = nx / 2;
    const int maxBlocksY = ny / 2;

    blockSamplesX = blockSamplesX > maxBlocksX ? maxBlocksX : blockSamplesX;
    blockSamplesY = blockSamplesY > maxBlocksY ? maxBlocksY : blockSamplesY;

    const int cx0 = ( ( W - nx ) / 2 / 2 ) * 2;
    const int cy0 = ( ( H - ny ) / 2 / 2 ) * 2;

    // const int strideX = ( nx - 2 ) / ( blockSamplesX - 1 );
    // const int strideY = ( ny - 2 ) / ( blockSamplesY - 1 );
    const int strideX = nx / blockSamplesX;
    const int strideY = ny / blockSamplesY;
    const int topX    = cx0 + strideX * blockSamplesX;
    const int topY    = cy0 + strideY * blockSamplesY;

    // Allocate memory.
    mN = blockSamplesX*2 * blockSamplesY*2;

    mX.reset( new int[mN] );
    mY.reset( new int[mN] );
    mC.reset( new xf[mN] );

    mBSX = blockSamplesX;
    mBSY = blockSamplesY;

    int idx = 0;

    for ( int i = cy0; i < topY; i += strideY )
    {
        for ( int j = cx0; j < topX; j += strideX )
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

xf CentralMeanBrightness::get_mean_brightness(cv::InputArray _img)
{
    // std::cout << "In CentralMeanBrightness::get_mean_brightness()" << std::endl;
    
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
        // Since mC is floating point type, the result of the expression will be promoted to floating point.
        meanBrightness += img.at<uint8_t>( mY[i], mX[i] ) * mC[i]; 
    }

    return meanBrightness / ( mN/4 );
}
