#ifndef __CENTRAL_MEAN_BRIGHTNESS_HPP__
#define __CENTRAL_MEAN_BRIGHTNESS_HPP__

#include <memory>
#include <string>

#include "AEAG/MeanBrightness.hpp"

namespace sxc
{

class CentralMeanBrightness : public MeanBrightness
{
public:
    /**
     * @param width The width of the target iamge.
     * @param height The height of the target image.
     * @param fX The ratio of width of the central area.
     * @param fY The ratio of height of the central area.
     * @param blockSamplesX The desired number of samples along the x-axis.
     * @param blockSamplesY The desired number of samples along the y-axis.
     * @param fR The red boost factor.
     * @param fG The green boost factor.
     * @param fB The blue boost factor.
     */
    CentralMeanBrightness(
        int width, int height, 
        xf fX, xf fY, int blockSamplesX, int blockSamplesY, 
        xf fR=1.0, xf fG=1.0, xf fB=1.0);
    virtual ~CentralMeanBrightness();

private:
    void fill_indices(int W, int H, xf fX, xf fY, int blockSamplesX, int blockSamplesY);

protected:
    xf get_mean_brightness(cv::InputArray _img);

protected:
    const xf CR;
    const xf CG;
    const xf CB;

protected:
    int  mBSX; // Block samples x.
    int  mBSY; // Block samples y.
    std::unique_ptr<int []> mX;  // X indices.
    std::unique_ptr<int []> mY;  // Y indices.
    std::unique_ptr<xf []>  mC;  // Coefficients.
    int  mN;  // Number of indices.
};

} // namespace sxc

#endif // __CENTRAL_MEAN_BRIGHTNESS_HPP__