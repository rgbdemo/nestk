#ifndef NESTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_RGBD_ICP_H
#define NESTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_RGBD_ICP_H

#include "relative_pose_estimator_icp.h"

namespace ntk
{

template <class PointT>
class RelativePoseEstimatorRGBDICP : public RelativePoseEstimatorICPWithNormals<PointT>
{
protected:
    typedef RelativePoseEstimatorICPWithNormals<PointT> super;
    typedef pcl::PointCloud<PointT> PointCloudType;
    typedef typename PointCloudType::ConstPtr PointCloudConstPtr;
    typedef typename PointCloudType::Ptr PointCloudPtr;

    using super::m_max_iterations;
    using super::m_distance_threshold;
    using super::m_ransac_outlier_threshold;

public:
    RelativePoseEstimatorRGBDICP()
        : target_points_3d(0),
          source_image_points(0)
    {}

    void setColorFeatures(const Pose3D& source_rgb_pose,
                          const std::vector<cv::Point3f>& target_points_3d,
                          const std::vector<cv::Point3f>& source_image_points)
    {
        this->source_rgb_pose = source_rgb_pose;
        this->target_points_3d = &target_points_3d;
        this->source_image_points = &source_image_points;
    }

protected:
    virtual bool computeRegistration(Pose3D& relative_pose,
                                     PointCloudConstPtr source_cloud,
                                     PointCloudConstPtr target_cloud,
                                     PointCloudType& aligned_cloud);

private:
    const std::vector<cv::Point3f>* target_points_3d;
    const std::vector<cv::Point3f>* source_image_points;
    Pose3D source_rgb_pose;
};

} // ntk

#endif // NESTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_RGBD_ICP_H
