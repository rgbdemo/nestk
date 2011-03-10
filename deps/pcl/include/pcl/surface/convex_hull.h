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
 * $Id: convex_hull.h 35810 2011-02-08 00:03:46Z rusu $
 *
 */

#ifndef PCL_CONVEX_HULL_2D_H_
#define PCL_CONVEX_HULL_2D_H_

// PCL includes
#include "pcl/pcl_base.h"
#include "pcl/io/io.h"

#include "pcl/features/normal_3d.h"
#include "pcl/ModelCoefficients.h"

namespace pcl
{
  /** \brief Sort points in a vector structure according to their .x/.y values
    * \param p1 the first point
    * \param p2 the second point
    */
  inline bool
  comparePoint2D (const Eigen::Vector4f &p1, const Eigen::Vector4f &p2)
  {
    if (p1[0] < p2[0])      return (true);
    else if (p1[0] > p2[0]) return (false);
    else if (p1[1] < p2[1]) return (true);
    else                    return (false);
  }

  void 
  convexHull2D (
      const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &points, 
      std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &hull);

  /** \brief Compute a 2D convex hull in 3D space using Andrew's monotone chain algorithm
    * \param cloud the point cloud message
    * \param indices the point indices to use from the cloud (they must form a planar model)
    * \param coeff the normalized planar model coefficients
    * \param hull the resultant convex hull model
    */
   template <typename PointT> void 
   convexHull2Dfrom3D (const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices, 
                       const Eigen::Vector4f &coeff, pcl::PointCloud<PointT> &hull);

  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b ConvexHull2D represents a 2D ConvexHull implementation.
    * \author Radu Bogdan Rusu
    */
  template <typename PointInT>
  class ConvexHull2D: public PCLBase<PointInT>
  {
    using PCLBase<PointInT>::input_;
    using PCLBase<PointInT>::indices_;
    using PCLBase<PointInT>::initCompute;
    using PCLBase<PointInT>::deinitCompute;

    public:
      typedef pcl::PointCloud<PointInT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      /** \brief Empty constructor. */
      ConvexHull2D () {};

      /** \brief Base method for surface reconstruction for all points given in <setInputCloud (), setIndices ()>
        * \param output the resultant reconstructed surface model
        */
      void 
      reconstruct (PointCloud &output);

    protected:
      /** \brief A pointer to the vector of model coefficients. */
      ModelCoefficientsConstPtr model_;

      /** \brief Abstract surface reconstruction method. */
      void 
      performReconstruction (PointCloud &output);

      /** \brief Abstract class get name method. */
      std::string 
      getClassName () const { return ("ConvexHull2D"); }
  };
}

#endif  //#ifndef PCL_CONVEX_HULL_2D_H_
