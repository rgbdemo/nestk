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
 * $Id: convex_hull.hpp 35830 2011-02-08 06:18:23Z rusu $
 *
 */

#ifndef PCL_SURFACE_IMPL_CONVEX_HULL_H_
#define PCL_SURFACE_IMPL_CONVEX_HULL_H_

#include "pcl/surface/convex_hull.h"

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::convexHull2Dfrom3D (const pcl::PointCloud<PointT> &cloud, 
                         const std::vector<int> &indices, 
                         const Eigen::Vector4f &coeff,
                         pcl::PointCloud<PointT> &hull)
{
 // Determine the best plane to project points onto
 int k0, k1, k2;
 k0 = (fabs (coeff[0] ) > fabs (coeff[1])) ? 0  : 1;
 k0 = (fabs (coeff[k0]) > fabs (coeff[2])) ? k0 : 2;
 k1 = (k0 + 1) % 3;
 k2 = (k0 + 2) % 3;

 // Compute a 2D centroid for two dimensions
 Eigen::Vector2d centroid = Eigen::Vector2d::Zero ();
 std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > epoints (indices.size ());
 for (size_t cp = 0; cp < indices.size (); ++cp)
 {
   epoints[cp][0] = cloud.points[indices[cp]].x;
   epoints[cp][1] = cloud.points[indices[cp]].y;
   epoints[cp][2] = cloud.points[indices[cp]].z;
   centroid[0] += epoints[cp][k1];
   centroid[1] += epoints[cp][k2];
 }
 centroid /= indices.size ();

 // Push projected centered 2d points
 std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > epoints_demean (epoints.size ());
 for (size_t cp = 0; cp < indices.size (); ++cp)
 {
   epoints_demean[cp][0] = epoints[cp][k1] - centroid[0];
   epoints_demean[cp][1] = epoints[cp][k2] - centroid[1];
 }

 std::sort (epoints_demean.begin (), epoints_demean.end (), comparePoint2D);

 std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > hull_2d;
 convexHull2D (epoints_demean, hull_2d);

 int nr_points_hull = hull_2d.size ();
 if (nr_points_hull >= 3)
 {
   // Determine the convex hull direction
   Eigen::Vector3f p1, p2, p3;

   p1[k0] = 0;
   p1[k1] = -hull_2d[0][0] + hull_2d[1][0];
   p1[k2] = -hull_2d[0][1] + hull_2d[1][1];

   p2[k0] = 0;
   p2[k1] = -hull_2d[0][0] + hull_2d[2][0];
   p2[k2] = -hull_2d[0][1] + hull_2d[2][1];

   p3 = p1.cross (p2);

   bool direction = (p3[k0] * coeff[k0] > 0);

   // Create the Polygon3D object
   hull.points.resize (nr_points_hull);

   // Copy hull points in clockwise or anti-clockwise format
   for (int cp = 0; cp < nr_points_hull; ++cp)
   {
     int d = direction ? cp : (nr_points_hull - cp - 1);
     Eigen::Vector3f pt;
     pt[k1] = hull_2d[cp][0] + centroid[0];
     pt[k2] = hull_2d[cp][1] + centroid[1];
     pt[k0] = -(coeff[3] + pt[k1] * coeff[k1] + pt[k2] * coeff[k2]) / coeff[k0];

     // Copy the point data to Polygon3D format
     hull.points[d].x = pt[0];
     hull.points[d].y = pt[1];
     hull.points[d].z = pt[2];
   }
 }
}


//////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::ConvexHull2D<PointInT>::reconstruct (PointCloud &output)
{
  output.header = input_->header;
  if (!initCompute ()) 
  {
    output.points.clear ();
    return;
  }

  // Perform the actual surface reconstruction
  performReconstruction (output);

  output.width    = output.points.size ();
  output.height   = 1;
  output.is_dense = true;

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
pcl::ConvexHull2D<PointInT>::performReconstruction (PointCloud &output)
{
  // Compute the plane coefficients (cheaper than waiting for them on the wire)
  Eigen::Vector4f model_coefficients;
  float curvature;
  computePointNormal<PointInT> (*input_, *indices_, model_coefficients, curvature);
  // Check if the plane coefficients were correctly estimated
  if (!pcl_isfinite (model_coefficients[0]) || 
      !pcl_isfinite (model_coefficients[1]) || 
      !pcl_isfinite (model_coefficients[2]) || 
      !pcl_isfinite (model_coefficients[3]))
  {
    ROS_WARN ("[pcl::%s::performReconstruction] Plane coefficients could not be estimated (nan/inf)!", getClassName ().c_str ());
    output.points.clear ();
    return;
  }
  //convexHull2Dfrom3D (*input_, *indices_, model_->values, output);
  convexHull2Dfrom3D (*input_, *indices_, model_coefficients, output);
}

#define PCL_INSTANTIATE_ConvexHull2D(T) template class pcl::ConvexHull2D<T>;
#define PCL_INSTANTIATE_convexHull2Dfrom3D(T) template void pcl::convexHull2Dfrom3D<T>(const pcl::PointCloud<T> &, const std::vector<int> &, const Eigen::Vector4f &, pcl::PointCloud<T> &);


#endif    // PCL_SURFACE_IMPL_CONVEX_HULL_H_

