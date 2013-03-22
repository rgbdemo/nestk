#ifndef NTK_GEOMETRY_RELATIVE_CLOUD_POSE_ESTIMATOR_PLANAR_H
#define NTK_GEOMETRY_RELATIVE_CLOUD_POSE_ESTIMATOR_PLANAR_H

#include "relative_pose_estimator.h"

#include <ntk/geometry/plane.h>

namespace ntk
{

/*!
 * Estimate the relative 3D pose between a new cloud and the reference one.
 *
 * The return pose is such that the new image, if projected to 3D using
 * the estimated pose, will result into an aligned point cloud.
 *
 * This is based on the PCL library implementation of ICP.
 * The algorithm first downsample the image using a voxel grid filter.
 */
class RelativePoseEstimatorTableTop : public RelativePoseEstimatorFromPointClouds<pcl::PointXYZ>
{
public:
    RelativePoseEstimatorTableTop()
        : m_distance_threshold(0.05),
          m_voxel_leaf_size(0.005),
          m_max_iterations(20)
    {}

public:
    void setSupportPlane(const ntk::Plane& plane) { m_plane = plane; }

    /*! Distance threshold to associate points. */
    void setDistanceThreshold(double th) { m_distance_threshold = th; }

    /*! Size of the leafs of the voxel grid. */
    void setVoxelSize(double s) { m_voxel_leaf_size = s; }

    /*! Set the maximal number of ICP iterations. */
    void setMaxIterations(int n) { m_max_iterations = n; }

public:
    virtual bool estimateNewPose();

protected:
    ntk::Plane m_plane;
    double m_distance_threshold;
    double m_voxel_leaf_size;
    int m_max_iterations;
};

} // ntk

#endif // NTK_GEOMETRY_RELATIVE_CLOUD_POSE_ESTIMATOR_PLANAR_H
