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

// For QTCreator
// #define NESTK_USE_PCL

#include "relative_pose_estimator.h"

#include <ntk/utils/time.h>
#include <ntk/utils/stl.h>
#include <ntk/stats/histogram.h>
#include <ntk/numeric/levenberg_marquart_minimizer.h>
#include <ntk/utils/opencv_utils.h>
#include <ntk/mesh/mesh.h>

#ifdef NESTK_USE_PCL
# include <pcl/registration/icp.h>
# include <ntk/mesh/pcl_utils.h>
# include <pcl/filters/voxel_grid.h>
# include <ntk/mesh/pcl_utils.h>
using namespace pcl;
#endif

using namespace cv;

namespace ntk
{

/*!
 * Compute the projection error of a 3D point cloud on a new image.
 * If feature points on the new image have depth information, it
 * will be taken into account.
 */
struct reprojection_error_3d : public ntk::CostFunction
{
  reprojection_error_3d(const Pose3D& initial_pose,
                        const std::vector<Point3f>& ref_points,
                        const std::vector<Point3f>& img_points)
    : CostFunction(6, ref_points.size()*3),
      initial_pose(initial_pose),
      ref_points(ref_points),
      img_points(img_points)
  {
    ntk_assert(ref_points.size() == img_points.size(), "Invalid matches");
  }

  virtual void evaluate (const std::vector< double > &x, std::vector< double > &fx) const
  {
    const bool use_depth = true;
    Pose3D new_pose = initial_pose;
    new_pose.applyTransformAfter(Vec3f(x[3],x[4],x[5]), cv::Vec3f(x[0],x[1],x[2]));
    int err_i = 0;
    std::fill(stl_bounds(fx), 0);
    foreach_idx(p_i, ref_points)
    {
      const Point3f& ref_point = ref_points[p_i];
      const Point3f& img_point = img_points[p_i];
      Point3f proj_p = new_pose.projectToImage(ref_point);
      fx[err_i*3] = (proj_p.x - img_point.x);
      fx[err_i*3+1] = (proj_p.y - img_point.y);
      if (use_depth && img_point.z > 1e-5 && ref_point.z > 1e-5)
        fx[err_i*3+2] = new_pose.meanFocal() * (proj_p.z - img_point.z);
      else
        fx[err_i*3+2] = 0;
      err_i = err_i + 1;
    }
  }

private:
  const Pose3D& initial_pose;
  const std::vector<Point3f>& ref_points;
  const std::vector<Point3f>& img_points;
};

// atomic mean square pose estimation.
double rms_optimize_3d(Pose3D& pose3d,
                       const std::vector<Point3f>& ref_points,
                       const std::vector<Point3f>& img_points)
{
  std::vector<double> fx;
  std::vector<double> initial(6);
  reprojection_error_3d f(pose3d, ref_points, img_points);
  LevenbergMarquartMinimizer optimizer;
  std::fill(stl_bounds(initial), 0);
  fx.resize(ref_points.size()*3);
  optimizer.minimize(f, initial);
  optimizer.diagnoseOutcome();
  f.evaluate(initial, fx);

  double error = f.outputNorm(initial);

  pose3d.applyTransformAfter(Vec3f(initial[3],initial[4],initial[5]), cv::Vec3f(initial[0], initial[1], initial[2]));
  return error;
}

double rms_optimize_ransac(Pose3D& pose3d,
                           const std::vector<Point3f>& ref_points,
                           const std::vector<Point3f>& img_points,
                           std::vector<bool>& valid_points)
{
  // One centimeter. 1000*1000 comes from the error scale factor
  // in rms_optimize.
  const double rms_err_threshold = 5;
  const double compat_err_threshold = 3;
  const int max_iterations = 200;
  const int min_support_points = std::max(7, int(ref_points.size()/20));
  const float min_consensus_support_percent = 0.05;

  ntk_assert(ref_points.size() > min_support_points, "Not enough points.");

  cv::RNG rgen;
  double best_error = FLT_MAX;
  Pose3D best_pose = pose3d;
  Pose3D current_pose = pose3d;
  std::vector<Point3f> current_ref_points;
  std::vector<Point3f> current_img_points;
  std::set<int> best_indices;
  std::set<int> indices;

  for (int it = 0; it < max_iterations && best_error > 0.01; ++it)
  {
    // initial set and model
    draw_k_different_numbers(rgen, indices, min_support_points, ref_points.size());

    current_pose = pose3d;
    current_ref_points.clear();
    current_img_points.clear();
    current_ref_points.reserve(indices.size());
    current_img_points.reserve(indices.size());
    foreach_const_it(it, indices, std::set<int>)
    {
      current_ref_points.push_back(ref_points[*it]);
      current_img_points.push_back(img_points[*it]);
    }

    double initial_error = rms_optimize_3d(current_pose, current_ref_points, current_img_points);
    ntk_dbg_print(initial_error, 2);

    // Base solution not good enough.
    if (initial_error > rms_err_threshold)
      continue;

    ntk_dbg_print(initial_error, 2);
    // determine consensus set.
    foreach_idx(index, ref_points)
    {
      if (indices.find(index) != indices.end())
        continue;

      Point3f proj_p = current_pose.projectToImage(ref_points[index]);
      double error = 0;
      error += ntk::math::sqr(proj_p.x - img_points[index].x);
      error += ntk::math::sqr(proj_p.y - img_points[index].y);
      if (img_points[index].z > 1e-5)
        error += ntk::math::sqr((proj_p.z - img_points[index].z)*current_pose.meanFocal());
      error = sqrt(error);

      if (error < compat_err_threshold)
        indices.insert(index);
    }

    ntk_dbg_print(indices.size(), 2);
    ntk_dbg_print(min_consensus_support_percent*ref_points.size(), 2);
    if (indices.size() < (min_consensus_support_percent*ref_points.size()))
      continue;

    current_ref_points.clear();
    current_img_points.clear();
    foreach_const_it(it, indices, std::set<int>)
    {
      current_ref_points.push_back(ref_points[*it]);
      current_img_points.push_back(img_points[*it]);
    }
    // current_pose = pose3d;

    double iteration_error = rms_optimize_3d(current_pose, current_ref_points, current_img_points);
    ntk_dbg_print(iteration_error, 2);
    if (iteration_error < best_error)
    {
      best_error = iteration_error;
      best_pose = current_pose;
      best_indices = indices;
    }
  } // end ransac loop

  valid_points.resize(ref_points.size(), false);
  foreach_const_it(it, best_indices, std::set<int>)
      valid_points[*it] = true;
  ntk_dbg_print(best_indices.size(), 2);
  pose3d = best_pose;
  return best_error;
}

} // ntk

namespace ntk
{

bool RelativePoseEstimatorFromFile::estimateNewPose(const RGBDImage& image)
{
  ntk_ensure(image.hasDirectory(), "Only works in fake mode!");
  m_current_pose.parseAvsFile((image.directory() + "/relative_pose.avs").c_str());
  return true;
}

bool RelativePoseEstimatorFromDelta::estimateNewPose(const RGBDImage& image)
{
  m_current_pose.applyTransformAfter(m_delta_pose);
  return true;
}

/*!
 * Compute the number of closest feature matches for each previous view.
 */
int RelativePoseEstimatorFromImage::
computeNumMatchesWithPrevious(const RGBDImage& image,
                              const FeatureSet& features,
                              std::vector<DMatch>& best_matches)
{
  const int min_number_of_matches = 30;
  int best_prev_image = 0;
  // If at least 30 matches have been found with one image, stop searching.
  // Start with the last one, more likely to have a similar point of view.
  for (int i = m_features.size()-1;
       i >= 0 && best_matches.size() < min_number_of_matches;
       --i)
  {
    std::vector<DMatch> current_matches;
    m_features[i].matchWith(features, current_matches, 0.8*0.8);
    ntk_dbg_print(current_matches.size(), 1);
    if (current_matches.size() > best_matches.size())
    {
      best_prev_image = i;
      best_matches = current_matches;
    }
  }
  return best_prev_image;
}

bool RelativePoseEstimatorFromImage::
estimateDeltaPose(Pose3D& new_rgb_pose,
                  const RGBDImage& image,
                  const FeatureSet& image_features,
                  const std::vector<cv::DMatch>& best_matches,
                  int closest_view_index)
{
  const float err_threshold = 5;

  ntk_dbg_print(new_rgb_pose, 2);
  const ImageData& ref_image_data = m_image_data[closest_view_index];

  ntk_dbg_print(best_matches.size(), 2);
  if (best_matches.size() < 10)
  {
    ntk_dbg(1) << "Not enough point matches (< 10)";
    return false;
  }

  std::vector<Point3f> ref_points;
  std::vector<Point3f> img_points;
  std::vector<KeyPoint> ref_keypoints;
  std::vector<KeyPoint> img_keypoints;

  foreach_idx(i, best_matches)
  {
    const DMatch& m = best_matches[i];
    const FeatureSet& ref_set = m_features[closest_view_index];
    const FeaturePoint& ref_loc = ref_set.locations()[m.trainIdx];
    const FeaturePoint& img_loc = image_features.locations()[m.queryIdx];

    ntk_assert(ref_loc.depth > 0, "Match without depth, should not appear");

    Point3f img3d (img_loc.pt.x,
                   img_loc.pt.y,
                   img_loc.depth);

    ref_points.push_back(ref_loc.p3d);
    img_points.push_back(img3d);
  }

  ntk_dbg_print(ref_points.size(), 2);
  if (ref_points.size() < 10)
  {
    ntk_dbg(2) << "Not enough matches with depth";
    return false;
  }

  // double error = rms_optimize_3d(new_pose, ref_points, img_points);
  std::vector<bool> valid_points;
  double error = rms_optimize_ransac(new_rgb_pose, ref_points, img_points, valid_points);

  ntk_dbg_print(error, 1);
  ntk_dbg_print(new_rgb_pose, 2);

  if (error < err_threshold)
    return true;
  else
    return false;
}

bool RelativePoseEstimatorFromImage::estimateNewPose(const RGBDImage& image)
{
  ntk_ensure(image.calibration(), "Image must be calibrated.");
  bool first_pass = false;
  if (!m_current_pose.isValid())
  {
    m_current_pose = *image.calibration()->depth_pose;
    first_pass = true;
  }

  ntk_ensure(image.mappedDepth().data, "Image must have depth mapping.");
  FeatureSet image_features;
  image_features.extractFromImage(image, m_feature_parameters);

  Pose3D new_pose = *image.calibration()->depth_pose;
  Pose3D new_rgb_pose = new_pose;
  new_rgb_pose.toRightCamera(image.calibration()->rgb_intrinsics,
                             image.calibration()->R, image.calibration()->T);
  bool pose_ok = true;

  int closest_view_index = -1;

  if (m_image_data.size() > 0)
  {
    std::vector<DMatch> best_matches;
    closest_view_index = computeNumMatchesWithPrevious(image, image_features, best_matches);
    ntk_dbg_print(closest_view_index, 1);
    ntk_dbg_print(best_matches.size(), 1);
#ifdef HEAVY_DEBUG
    cv::Mat3b debug_img;
    m_features[closest_view_index].drawMatches(m_image_data[closest_view_index].color, image.rgb(), image_features, best_matches, debug_img);
    imwrite("/tmp/debug_matches.png", debug_img);
#endif

    new_pose = m_image_data[closest_view_index].depth_pose;
    new_rgb_pose = new_pose;
    new_rgb_pose.toRightCamera(image.calibration()->rgb_intrinsics,
                               image.calibration()->R, image.calibration()->T);

    if (best_matches.size() > 0)
    {
      // Estimate the relative pose w.r.t the closest view.
      if (!estimateDeltaPose(new_rgb_pose, image, image_features, best_matches, closest_view_index))
        pose_ok = false;
      new_pose = new_rgb_pose;
      new_pose.toLeftCamera(image.calibration()->depth_intrinsics,
                            image.calibration()->R, image.calibration()->T);

    }
    else
    {
      pose_ok = false;
    }
  }

  if (pose_ok)
  {
    if (m_use_icp)
      pose_ok &= optimizeWithICP(image, new_pose, closest_view_index);
  }

  if (pose_ok)
  {
    new_rgb_pose = new_pose;
    new_rgb_pose.toRightCamera(image.calibration()->rgb_intrinsics,
                               image.calibration()->R, image.calibration()->T);


    m_current_pose = new_pose;
    if (m_incremental_model || first_pass)
    {
        ImageData image_data;
        image.rgb().copyTo(image_data.color);
        image_data.depth_pose = new_pose;
        image_features.compute3dLocation(new_rgb_pose);
        m_features.push_back(image_features);
        ntk_dbg_print(image_features.locations().size(), 1);
        m_image_data.push_back(image_data);
    }
    return true;
  }
  else
    return false;
}

void RelativePoseEstimatorFromImage::reset()
{
  m_features.clear();
  m_image_data.clear();
  m_current_pose = Pose3D();
}

#ifdef NESTK_USE_PCL

bool RelativePoseEstimatorFromImage::optimizeWithICP(const RGBDImage& image,
                                                     Pose3D& depth_pose,
                                                     int closest_view_index)
{
  const int min_ref_points = 100;

  PointCloud<PointXYZIndex> cloud_source, cloud_target, cloud_reg;
  bool pose_ok=false;

  Pose3D new_depth_pose = depth_pose;
  rgbdImageToPointCloud(cloud_source, image, new_depth_pose, 4);

  // Using as reference points features points from views adjacent to the best
  // matching one to keep it small.
  std::vector<Point3f> ref_points;
  // Add points for adjacent views.
  for (int i = closest_view_index - 1; i <= closest_view_index + 1; ++i)
  {
    if (i < 0 || i >= m_features.size()) continue;
    for (int k = 0; k < m_features[i].locations().size(); ++k)
      ref_points.push_back(m_features[i].locations()[k].p3d);
  }

  if (ref_points.size() < min_ref_points)
  {
    ntk_dbg(1) << "Not enough reference points, ignoring ICP.";
    return true;
  }

  vectorToPointCloud(cloud_target, ref_points);

  pcl::PointCloud<PointXYZIndex>::ConstPtr cloud_source_ptr = cloud_source.makeShared();
  pcl::PointCloud<PointXYZIndex>::ConstPtr cloud_target_ptr = cloud_target.makeShared();

  ntk_dbg_print(cloud_source.points.size(), 1);
  ntk_dbg_print(cloud_target.points.size(), 1);

  IterativeClosestPoint<PointXYZIndex, PointXYZIndex> reg;

  PointCloud<PointXYZIndex> cloud_filtered_target;
  PointCloud<PointXYZIndex> cloud_filtered_source;

  // Simplify the point clouds.
  VoxelGrid<PointXYZIndex> grid;
  grid.setLeafSize (0.02, 0.02, 0.02);

  grid.setInputCloud (cloud_target_ptr);
  grid.filter (cloud_filtered_target);

  grid.setInputCloud (cloud_source_ptr);
  grid.filter (cloud_filtered_source);

  reg.setInputCloud (cloud_filtered_target.makeShared() );
  reg.setInputTarget (cloud_filtered_source.makeShared()  );

  reg.setMaximumIterations (20);
  reg.setTransformationEpsilon (1e-5);
  reg.setMaxCorrespondenceDistance (0.01);

  reg.align (cloud_reg);
  if (!reg.hasConverged())
  {
    ntk_dbg(1) << "ICP did not converge, ignoring.";
    return false;
  }

  Eigen::Matrix4f t = reg.getFinalTransformation ();
  cv::Mat1f T(4,4);
  //toOpencv(t,T);
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      T(r,c) = t(r,c);

  Pose3D icp_pose;
  icp_pose = new_depth_pose;
  icp_pose.setCameraTransform(T);

  new_depth_pose.applyTransformAfter(icp_pose);
  depth_pose = new_depth_pose;
  return true;
}

#else // NESTK_USE_PCL

bool RelativePoseEstimatorFromImage::optimizeWithICP(const RGBDImage& image, Pose3D& depth_pose, int closest_view_index)
{
  return false;
}

#endif // NESTK_USE_PCL

} // ntk
