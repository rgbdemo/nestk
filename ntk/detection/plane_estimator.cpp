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
 * Author: Jorge Garcia Bueno <jgarcia@ing.uc3m.es>, (C) 2010
 */

# include "plane_estimator.h"

# include <ntk/ntk.h>
# include <ntk/geometry/pose_3d.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

#ifdef NESTK_USE_PCL
# include <ntk/mesh/pcl_utils.h>
# include "pcl/sample_consensus/method_types.h"
# include "pcl/sample_consensus/model_types.h"
# include "pcl/segmentation/sac_segmentation.h"

using namespace pcl;
#endif

using namespace cv;

namespace ntk
{

  double PlaneSolver::EnergyFunction(double *trial,bool &bAtSolution)
  {
    double distTotal = 0.0;
    double distRel = 0.0;

    for (int i = 0; i < g.size(); i++)
    {
       // Norm 2
       // distRel = (trial[0] * g[i].x + trial[1] * g[i].y + trial[2] * g[i].z + trial[3])/
       //            sqrt(trial[0] * trial[0] +  trial[1] * trial[1] +  trial[2] * trial[2]);

       // Norm 1
         distRel = (trial[0] * g[i].x + trial[1] * g[i].y + trial[2] * g[i].z + trial[3])/
                   (fabs(trial[0]) + fabs(trial[1]) +  fabs(trial[2]));
         //std::cout << distRel << std::endl;

           distTotal+=((double)fabs(distRel));
    }
    return(distTotal);
  }



  PlaneEstimator :: PlaneEstimator()
    : m_solver(dim, population),
      m_ref_plane(0.0,1.0,1.0)
  {
    ntk::normalize(m_ref_plane);
    double min[dim];
    double max[dim];
    int i;

    for (i=0;i<dim;i++)
    {
      max[i] = 1000.0;
      min[i] = -1000.0;
    }
    m_solver.Setup(min,max,DifferentialEvolutionSolver::stBest1Exp,0.8,0.75);
  }

  void PlaneEstimator :: estimate(RGBDImage& image, Mat1b& plane_points)
  {

      // Passing from 3D to the optimizer

      const cv::Mat3f& normal_image = image.normal();
      const cv::Mat1f& distance_image = image.depth();
      cv::Mat1b& mask_image = image.depthMaskRef();
      cv::Mat1b objfilter;
      mask_image.copyTo(objfilter);
      plane_points = image.normal().clone();
      plane_points = 0;

    if (!image.normal().data)
    {
      ntk_dbg(0) << "WARNING: you must active the normal filter to get plane estimation!";
      return;
    }

    double min[dim];
    double max[dim];
    int i;

    for (i=0;i<dim;i++)
    {
      max[i] = 1000.0;
      min[i] = -1000.0;
    }
    m_solver.Setup(min,max,DifferentialEvolutionSolver::stBest1Exp,0.8,0.75);


   // Early estimation of plane points projecting the normal values

    for (int r = 1; r < plane_points.rows-1; ++r)
    for (int c = 1; c < plane_points.cols-1; ++c)
    {
      if (objfilter.data && objfilter(r,c))
      {
        cv::Vec3f normal = normal_image(r,c);
        double prod = normal.dot(m_ref_plane);
        if (prod > 0.95)
          plane_points(r,c) = 255;
        else
          plane_points(r,c) = 0;
      }
    }

    // cleaning of the surface very first estimation
    dilate(plane_points,plane_points,cv::Mat());
    erode(plane_points,plane_points,cv::Mat());
    //imwrite("plane-initial.png",plane_points);

    std:vector<Point3f>& g = m_solver.planePointsRef();

    g.clear();
    
    for (int r = 1; r < plane_points.rows-1; ++r)
    for (int c = 1; c < plane_points.cols-1; ++c)
    {
      if (plane_points(r,c))
      {
        // possible table member!
        Point3f p3d = image.calibration()->depth_pose->unprojectFromImage(Point2f(c,r), distance_image(r,c));
        g.push_back(p3d);
      }
    }
    
    // Calculating...
    m_solver.Solve(max_generations);

    double *solution = m_solver.Solution();

    // Plane normalizer
    float suma = solution[0] + solution[1] + solution[2] + solution[3] ;
    for (int i = 0; i < 4; i++)
      solution[i] = solution[i]/ suma;

    ntk::Plane plano (solution[0], solution[1], solution[2], solution[3]);
    //Update RGBD object
    m_plane.set(solution[0], solution[1], solution[2], solution[3]);


    // Final estimation of plane points projecting the normal values

     cv::Vec3f diffevolnormal(solution[0], solution[1], solution[2]);

     for (int r = 1; r < plane_points.rows-1; ++r)
     for (int c = 1; c < plane_points.cols-1; ++c)
     {
       if (objfilter.data && objfilter(r,c))
       {
         cv::Vec3f normal = normal_image(r,c);
         double prod = normal.dot(diffevolnormal);

         if (prod > 0.5)
           plane_points(r,c) = 255;
         else
           plane_points(r,c) = 0;
       }
     }

     // Cleaning of the DE plane-pixels solution
     dilate(plane_points,plane_points,cv::Mat());
     erode(plane_points,plane_points,cv::Mat());
    // imwrite("plane-DE.png",plane_points);


  }

} // ntk

namespace ntk
{

#ifdef NESTK_USE_PCL
void PclPlaneEstimator :: estimate(const ntk::RGBDImage& image, cv::Mat1b& plane_points, const Pose3D& pose)
{
  PointCloud<PointXYZIndex>::Ptr cloud(new PointCloud<PointXYZIndex>);
  rgbdImageToPointCloud(*cloud, image, pose);

  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  // Create the segmentation object
  pcl::SACSegmentation<PointXYZIndex> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  pcl::PointCloud<PointXYZIndex>::Ptr cloudptr = cloud; // FIXME: Find out whether the removal of the (deep-copying) cloud.makeShared() call sped things up.
  seg.setInputCloud (cloudptr);
  seg.segment (inliers, coefficients);

  m_plane = ntk::Plane (coefficients.values[0],
                        coefficients.values[1],
                        coefficients.values[2],
                        coefficients.values[3]);

  foreach_idx(i, inliers.indices)
  {
    PointXYZIndex p = cloud->points[inliers.indices[i]];
    int r = p.rgba / image.depth().cols;
    int c = p.rgba % image.depth().cols;
    plane_points(r,c) = 255;
  }
}

#else // NESTK_USE_PCL

void PclPlaneEstimator :: estimate(const ntk::RGBDImage& image, cv::Mat1b& plane_points, const Pose3D& pose)
{
  ntk_throw_exception("PCL support not enabled!");
}

#endif // NESTK_USE_PCL

} // ntk
