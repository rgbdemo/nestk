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
 *
 */

/* \author Bastian Steder */

#ifndef PCL_NARF_KEYPOINT_H_
#define PCL_NARF_KEYPOINT_H_

#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/keypoint.h>

namespace pcl {

// Forward declarations
class RangeImage;
class RangeImageBorderExtractor;

/** \brief @b NARF (Normal Aligned Radial Feature) keypoints. Input is a range image, output the indices of the keypoints
  * \author Bastian Steder
  */
class NarfKeypoint : public Keypoint<PointWithRange, int>
{
  public:
    // =====TYPEDEFS=====
    typedef Keypoint<PointWithRange, int> BaseClass;
    
    typedef Keypoint<PointWithRange, int>::PointCloudOut PointCloudOut;

    // =====PUBLIC STRUCTS=====
    //! Parameters used in this class
    struct Parameters
    {
      Parameters() : support_size(-1.0f), interest_image_blur_size(1), min_distance_between_interest_points(0.1f),
                     optimal_distance_to_high_surface_change(0.25), min_pixel_distance_between_interest_points(2),
                     min_interest_value(0.1f), distance_for_additional_points(0.0f),
                     no_of_direction_pairs_to_average(3), add_points_on_straight_edges(false),
                     do_non_maximum_suppression(true) {}
      
      float support_size;  //!< This defines the area 'covered' by an interest point (in meters)
      int interest_image_blur_size;  //!< Radius for blurring used on th interest image
      float min_distance_between_interest_points;  /**< Minimum distance between maximas
                                                     *  (this a factor for support_size, i.e. the distance is
                                                     *  min_distance_between_interest_points*support_size) */
      float optimal_distance_to_high_surface_change;  /**< The distance we want keep between keypoints and areas
                                                        *  of high surface change
                                                        *  (this a factor for support_size, i.e., the distance is
                                                        *  optimal_distance_to_high_surface_change*support_size) */
      float min_pixel_distance_between_interest_points;  //!< Minimum distance between maximas in pixels
      float min_interest_value;  //!< The minimum value to consider a point as an interest point
      float distance_for_additional_points;  /**< All points in this distance to a found maximum, that
                                               *  are above min_interest_value are also added as interest points
                                               *  (this a factor for support_size, i.e. the distance is
                                               *  distance_for_additional_points*support_size) */
      int no_of_direction_pairs_to_average;  /**< During the calculation of the interest values, pairs of
                                               *  directions are evaluated   and the best n averaged */
      bool add_points_on_straight_edges;  /**< If this is set to true, there will also be interest points on
                                            *   straight edges, e.g., just indicating an area of high surface change */
      bool do_non_maximum_suppression;  /**< If this is set to false there will be much more points
                                          *  (can be used to spread points over the whole scene
                                          *  (combined with a low min_interest_value)) */
    };
    
    // =====CONSTRUCTOR & DESTRUCTOR=====
    NarfKeypoint (RangeImageBorderExtractor* range_image_border_extractor=NULL, float support_size=-1.0f);
    ~NarfKeypoint ();
    
    // =====PUBLIC METHODS=====
    //! Erase all data calculated for the current range image
    void
      clearData ();
    
    //! Set the RangeImageBorderExtractor member (required)
    void
      setRangeImageBorderExtractor (RangeImageBorderExtractor* range_image_border_extractor);
    
    //! Set the RangeImage member of the RangeImageBorderExtractor
    void
      setRangeImage (const RangeImage* range_image);
    
    /** Extract interest value per image point */
    float*
      getInterestImage () { calculateInterestImage(); return interest_image_;}

    //! Extract maxima from an interest image
    const ::pcl::PointCloud<InterestPoint>&
      getInterestPoints () { calculateInterestPoints(); return *interest_points_;}
    
    //! Set all points in the image that are interest points to true, the rest to false
    const std::vector<bool>&
      getIsInterestPointImage () { calculateInterestPoints(); return is_interest_point_image_;}
    
    //! Getter for the parameter struct
    Parameters&
      getParameters () { return parameters_;}

    //! Getter for the range image of range_image_border_extractor_
    const RangeImage&
      getRangeImage ();
    
    //! Overwrite the compute function of the base class
    void
      compute (PointCloudOut& output);
    
  protected:
    // =====PROTECTED METHODS=====
    void
      calculateInterestImage ();
    void
      calculateInterestPoints ();
    void
      blurInterestImage ();
    //! Detect key points
    virtual void
      detectKeypoints (PointCloudOut& output);
    
    // =====PROTECTED MEMBER VARIABLES=====
    using BaseClass::name_;
    RangeImageBorderExtractor* range_image_border_extractor_;
    Parameters parameters_;
    float* interest_image_;
    ::pcl::PointCloud<InterestPoint>* interest_points_;
    std::vector<bool> is_interest_point_image_;
};

inline std::ostream&
  operator << (std::ostream& os, const NarfKeypoint::Parameters& p)
{
  os << PVARC(p.support_size) << PVARC(p.min_distance_between_interest_points)
     << PVARC(p.min_pixel_distance_between_interest_points)
     << PVARC(p.min_interest_value) << PVARC(p.distance_for_additional_points)
     << PVARN(p.no_of_direction_pairs_to_average);
  return (os);
}

}  // end namespace pcl

#endif  //#ifndef PCL_NARF_KEYPOINT_H_
