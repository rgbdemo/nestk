/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
\author Bastian Steder
**/


#ifndef PCL_RANGE_IMAGE_PLANAR_H_
#define PCL_RANGE_IMAGE_PLANAR_H_

#include <pcl/range_image/range_image.h>

namespace pcl
{
  /** \brief @b RangeImagePlanar is derived from the original range image and differs from it because it's not a 
    * spherical projection, but using a projection plane (as normal cameras do), therefore being better applicable 
    * for range sensors that already provide a range image by themselves (stereo cameras, kinect, ToF-cameras), so that
    * a conversion to point cloud and then to a spherical range image becomes unnecessary.
    * \author Bastian Steder 
    */
  class RangeImagePlanar : public RangeImage
  {
    public:
      // =====CONSTRUCTOR & DESTRUCTOR=====
      /** Constructor */
      RangeImagePlanar ();
      /** Destructor */
      ~RangeImagePlanar ();
      
      // =====PUBLIC METHODS=====
      /** \brief Create the image from an existing disparity image.
        * \param disparity_image the input disparity image data
        * \param di_width the disparity image width
        * \param di_height the disparity image height
        * \param focal_length the focal length of the primary camera that generated the disparity image
        * \param base_line the baseline of the stereo pair that generated the disparity image
        * \param desired_angular_resolution If this is set, the system will skip as many pixels as necessary to get as
        *         close to this angular resolution as possible while not going over this value (the density will not be
        *         lower than this value). The value is in radians per pixel. 
        */
      void
      setDisparityImage (const float* disparity_image, int di_width, int di_height,
                         float focal_length, float base_line, float desired_angular_resolution=-1);
      
      /** Create the image from an existing depth image.
        * \param depth_image the input disparity image data
        * \param di_width the disparity image width 
        * \param di_height the disparity image height
        * \param di_center_x the x-coordinate of the camera's center of projection
        * \param di_center_y the y-coordinate of the camera's center of projection
        * \param di_focal_length_x the camera's focal length in the horizontal direction
        * \param di_focal_length_y the camera's focal length in the vertical direction
        * \param desired_angular_resolution If this is set, the system will skip as many pixels as necessary to get as
        *         close to this angular resolution as possible while not going over this value (the density will not be
        *         lower than this value). The value is in radians per pixel.
        */
      void
      setDepthImage (const float* depth_image, int di_width, int di_height, float di_center_x, float di_center_y,
                     float di_focal_length_x, float di_focal_length_y, float desired_angular_resolution=-1);
      
      // Since we reimplement some of these overloaded functions, we have to do the following:
      using RangeImage::calculate3DPoint;
      using RangeImage::getImagePoint;
      
      /** \brief Calculate the 3D point according to the given image point and range
        * \param image_x the x image position
        * \param image_y the y image position
        * \param range the range
        * \param point the resulting 3D point
        * \note Implementation according to planar range images (compared to spherical as in the original)
        */
      virtual inline void
      calculate3DPoint (float image_x, float image_y, float range, Eigen::Vector3f& point) const;
      
      /** \brief Calculate the image point and range from the given 3D point
        * \param point the resulting 3D point
        * \param image_x the resulting x image position
        * \param image_y the resulting y image position
        * \param range the resulting range
        * \note Implementation according to planar range images (compared to spherical as in the original)
        */
      virtual inline void 
      getImagePoint (const Eigen::Vector3f& point, float& image_x, float& image_y, float& range) const;
      
    protected:
      float focal_length_x_, focal_length_y_; //!< The focal length of the image in pixels
      float focal_length_x_reciprocal_, focal_length_y_reciprocal_;  //!< 1/focal_length -> for internal use
      float center_x_, center_y_;      //!< The principle point of the image
  };
}  // namespace end


#include "pcl/range_image/range_image_planar.hpp"  // Definitions of templated and inline functions

#endif  //#ifndef PCL_RANGE_IMAGE_H_
