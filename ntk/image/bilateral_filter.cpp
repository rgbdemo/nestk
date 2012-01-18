/**
 * This file is part of the nestk library.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2010
 */

#include "bilateral_filter.h"

#include <ntk/utils/debug.h>

using namespace cv;

namespace ntk
{

void
depth_bilateralFilter( const Mat1f& src_, Mat1f& dst_,
                       int d, double sigma_color, double sigma_space,
                       float maximal_delta_depth_percent,
                       int borderType )
{
    Mat src = src_;
    Mat dst = dst_;
    dst.create( src.size(), src.type() );
    dst_ = dst;


    int cn = src.channels();
    int i, j, k, maxk, radius;
    double minValSrc=-1, maxValSrc=1;
    const int kExpNumBinsPerChannel = 1 << 12;
    int kExpNumBins = 0;
    float lastExpVal = 1.f;
    float len, scale_index;
    Size size = src.size();

    CV_Assert( (src.type() == CV_32FC1 || src.type() == CV_32FC3) &&
               src.type() == dst.type() && src.size() == dst.size() &&
               src.data != dst.data );

    if( sigma_color <= 0 )
        sigma_color = 1;
    if( sigma_space <= 0 )
        sigma_space = 1;

    double gauss_color_coeff = -0.5/(sigma_color*sigma_color);
    double gauss_space_coeff = -0.5/(sigma_space*sigma_space);

    if( d <= 0 )
        radius = cvRound(sigma_space*1.5);
    else
        radius = d/2;
    radius = MAX(radius, 1);
    d = radius*2 + 1;
    // compute the min/max range for the input image (even if multichannel)

    minMaxLoc( src.reshape(1), &minValSrc, &maxValSrc );

    // temporary copy of the image with borders for easy processing
    Mat temp;
    copyMakeBorder( src, temp, radius, radius, radius, radius, borderType );

    // allocate lookup tables
    vector<float> _space_weight(d*d);
    vector<int> _space_ofs(d*d);
    float* space_weight = &_space_weight[0];
    int* space_ofs = &_space_ofs[0];

    // assign a length which is slightly more than needed
    len = (float)(maxValSrc - minValSrc) * cn;
    kExpNumBins = kExpNumBinsPerChannel * cn;
    vector<float> _expLUT(kExpNumBins+2);
    float* expLUT = &_expLUT[0];

    scale_index = kExpNumBins/len;

    // initialize the exp LUT
    for( i = 0; i < kExpNumBins+2; i++ )
    {
        if( lastExpVal > 0.f )
        {
            double val =  i / scale_index;
            expLUT[i] = (float)std::exp(val * val * gauss_color_coeff);
            lastExpVal = expLUT[i];
        }
        else
            expLUT[i] = 0.f;
    }

    // initialize space-related bilateral filter coefficients
    for( i = -radius, maxk = 0; i <= radius; i++ )
        for( j = -radius; j <= radius; j++ )
        {
            double r = std::sqrt((double)i*i + (double)j*j);
            if( r > radius )
                continue;
            space_weight[maxk] = (float)std::exp(r*r*gauss_space_coeff);
            space_ofs[maxk++] = (int)(i*(temp.step/sizeof(float)) + j*cn);
        }

    for( i = 0; i < size.height; i++ )
    {
        const float* sptr = (const float*)(temp.data + (i+radius)*temp.step) + radius*cn;
        float* dptr = (float*)(dst.data + i*dst.step);

        assert( cn == 1 );

        for( j = 0; j < size.width; j++ )
        {
            float sum = 0, wsum = 0;
            float val0 = sptr[j];
            float max_delta_depth = val0 * maximal_delta_depth_percent;
            if (val0 < 1e-5f) // invalid depth
            {
                dptr[j] = 0;
            }
            else
            {
                for( k = 0; k < maxk; k++ )
                {
                    float val = sptr[j + space_ofs[k]];
                    if (val < 1e-5f) // invalid depth
                        continue;

                    float delta_depth = std::abs(val - val0);
                    if (delta_depth > max_delta_depth)
                        continue;

                    float alpha = (float)(delta_depth*scale_index);
                    int idx = cvFloor(alpha);
                    alpha -= idx;
                    float w = space_weight[k]*(expLUT[idx] + alpha*(expLUT[idx+1] - expLUT[idx]));
                    sum += val*w;
                    wsum += w;
                }
                dptr[j] = (float)(sum/wsum);
            }
        }
    }
}

} // ntk
