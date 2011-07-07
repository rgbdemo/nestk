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

#ifndef NESTK_USE_PCL
# error NESTK_USE_PCL should be defined!
# define NESTK_USE_PCL
#endif

#include "relative_pose_estimator_icp.h"
#include <ntk/mesh/pcl_utils.h>

#include <pcl/registration/icp.h>

using namespace pcl;
using namespace cv;

namespace ntk
{

void RelativePoseEstimatorICP :: setReferenceImage(const RGBDImage& ref_image)
{
    rgbdImageToPointCloud(m_ref_cloud, ref_image);
}

bool RelativePoseEstimatorICP :: estimateNewPose(const RGBDImage& image)
{
    if (m_ref_cloud.points.size() < 1)
    {
        ntk_dbg(1) << "Reference cloud was empty";
        return false;
    }

    PointCloud<PointXYZ> cloud;
    rgbdImageToPointCloud(cloud, image);

    PointCloud<PointXYZ>::ConstPtr cloud_source_ptr = cloud.makeShared();
    PointCloud<PointXYZ>::ConstPtr cloud_target_ptr = m_ref_cloud.makeShared();
    PointCloud<PointXYZ> cloud_reg;

    IterativeClosestPoint<PointXYZ, PointXYZ> reg;
    reg.setMaximumIterations (20);
    reg.setTransformationEpsilon (1e-5);
    reg.setMaxCorrespondenceDistance (0.02);
    reg.setInputCloud (cloud_source_ptr);
    reg.setInputTarget (cloud_target_ptr);
    reg.align (cloud_reg);

    if (!reg.hasConverged())
    {
      ntk_dbg(1) << "ICP did not converge, ignoricloudng.";
      return false;
    }

    Eigen::Matrix4f t = reg.getFinalTransformation ();
    cv::Mat1f T(4,4);
    //toOpencv(t,T);
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        T(r,c) = t(r,c);

    Pose3D icp_pose;
    icp_pose.setCameraTransform(T);

    ntk_dbg_print(icp_pose.cvTranslation(), 1);

    m_current_pose.resetCameraTransform();
    m_current_pose = *image.calibration()->depth_pose;
    m_current_pose.applyTransformAfter(icp_pose);

    ntk_dbg_print(m_current_pose.cvTranslation(), 1);
    return true;
}

}
