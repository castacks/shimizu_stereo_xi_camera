#ifndef __DOWNSAMPLED_MEAN_BRIGHTNESS_HPP__
#define __DOWNSAMPLED_MEAN_BRIGHTNESS_HPP__

#include <string>

#include "AEAG/MeanBrightness.hpp"

namespace sxc
{

class DownSampledMeanBrightness : public MeanBrightness
{
public:
    DownSampledMeanBrightness(
        int nx, int ny, int blockSamplesX, int blockSamplesY, 
        xf fR=1.0, xf fG=1.0, xf fB=1.0);
    virtual ~DownSampledMeanBrightness();

private:
    void destroy(void);
    void fill_indices(int nx, int ny, int blockSamplesX, int blockSamplesY);

protected:
    xf get_mean_brightness(cv::InputArray _img);

protected:
    const xf CR;
    const xf CG;
    const xf CB;

protected:
    int  mBSX; // Block samples x.
    int  mBSY; // Block samples y.
    int *mX;  // X indices.
    int *mY;  // Y indices.
    xf  *mC;  // Coefficients.
    int  mN;  // Number of indices.
};

} // namespace sxc

#endif // __DOWNSAMPLED_MEAN_BRIGHTNESS_HPP__