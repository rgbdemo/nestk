#ifndef NTK_IMAGE_BILATERAL_FILTER_H
#define NTK_IMAGE_BILATERAL_FILTER_H

# include <ntk/core.h>

namespace ntk
{

/*! sigma_color is in pixels, sigma_space in meters. */
void depth_bilateralFilter (const cv::Mat1f& src, cv::Mat1f& dst, float sigma_color, float sigma_space);

void depth_bilateralFilter( const cv::Mat1f& src, cv::Mat1f& dst,
                            int d, double sigma_color, double sigma_space,
                            float maximal_delta_depth_percent = 0.005, // 5mm @ 1meter
                            int borderType = cv::BORDER_DEFAULT);

} // ntk

#endif // NTK_IMAGE_BILATERAL_FILTER_H
