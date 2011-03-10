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
 * $Id: convex_hull.cpp 35810 2011-02-08 00:03:46Z rusu $
 *
 */

#include "pcl/impl/instantiate.hpp"
#include "pcl/point_types.h"
#include "pcl/surface/convex_hull.h"
#include "pcl/surface/impl/convex_hull.hpp"

//////////////////////////////////////////////////////////////////////////
void
pcl::convexHull2D (const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &points, 
                   std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &hull)
{
   int nr_points = points.size ();
   hull.resize (nr_points + 1);

   // Indices for bottom and top of the stack
   int bot = 0, top = -1;
   int i;

   for (i = 1; i < nr_points; i++)
     // points[0].x represents the smallest X coordinate
     if (points[i][0] != points[0][0])
       break;

   // Get the indices of points with min|max y-coord
   int minmax = i - 1;

   // Degenerate case: all x-coords == xmin
   if ( minmax == (nr_points - 1) )
   {
     ++top;
     hull[top] = points[0];
     // A nontrivial segment
     if (points[minmax][1] != points[0][1])
     {
       ++top;
       hull[top] = points[minmax];
     }
     ++top;
     // Add the polygon's endpoint
     hull[top] = points[0];
     hull.resize (top + 1);
     return;
   }

   int maxmin;
   for (i = nr_points - 2; i >= 0; i--)
     if (points[i][0] != points[nr_points - 1][0])
       break;
   maxmin = i + 1;

   // Compute the lower hull
   ++top;
   // Add the polygon's endpoint
   hull[top] = points[0];

   i = minmax;
   while (++i <= maxmin)
   {
     // The lower line joins P[minmin] with P[maxmin]
     if ((i < maxmin) && (
         (points[maxmin][0] - points[0][0]) * (points[i][1]      - points[0][1]) -
         (points[i][0]      - points[0][0]) * (points[maxmin][1] - points[0][1]) >= 0))
       continue;          // ignore P[i] above or on the lower line

     // If there are at least 2 points on the stack
     while (top > 0)
     {
       // Test if P[i] is left of the line at the stack top
       if ((hull[top][0] - hull[top-1][0]) * (points[i][1] - hull[top-1][1]) -
           (points[i][0] - hull[top-1][0]) * (hull[top][1] - hull[top-1][1]) > 0)
         break;         // P[i] is a new hull vertex
       else
         top--;         // pop top point off stack
     }
     ++top;
     hull[top] = points[i];
   }

   // Next, compute the upper hull above the bottom hull
   if ((nr_points - 1) != maxmin)      // if distinct xmax points
   {
     ++top;
     // Add the point with max X and max Y coordinates to the hull
     hull[top] = points[nr_points - 1];
   }
   // The bottom point of the upper hull stack
   bot = top;

   i = maxmin;
   while (--i >= minmax)
   {
     // The upper line joins P[nr_points - 1] with P[minmax]
     if ((i > minmax) && (
         (points[minmax][0] - points[nr_points - 1][0]) * (points[i][1]      - points[nr_points - 1][1]) -
         (points[i][0]      - points[nr_points - 1][0]) * (points[minmax][1] - points[nr_points - 1][1]) >= 0))
       continue;          // ignore P[i] below or on the upper line

     // If there are at least 2 points on the stack
     while (top > bot)
     {
       // Test if P[i] is left of the line at the stack top
       if ((hull[top][0] - hull[top-1][0]) * (points[i][1] - hull[top-1][1]) -
           (points[i][0] - hull[top-1][0]) * (hull[top][1] - hull[top-1][1]) > 0)
         break;         // P[i] is a new hull vertex
       else
         top--;         // pop top point off stack
     }
     ++top;

     hull[top] = points[i];
   }

   if (minmax != 0)
   {
     ++top;
     // Add the polygon's endpoint
     hull[top] = points[0];
   }
   hull.resize (top + 1);
   return;
}

// Instantiations of specific point types
PCL_INSTANTIATE(ConvexHull2D, PCL_XYZ_POINT_TYPES);
PCL_INSTANTIATE(convexHull2Dfrom3D, PCL_XYZ_POINT_TYPES);

