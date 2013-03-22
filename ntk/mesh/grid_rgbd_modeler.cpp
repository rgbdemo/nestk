
#include "grid_rgbd_modeler.h"
#include <ntk/utils/opencv_utils.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/numeric/levenberg_marquart_minimizer.h>
#include <ntk/utils/time.h>
#include <ntk/mesh/pcl_utils.h>

#include <pcl/filters/approximate_voxel_grid.h>

using namespace cv;

namespace ntk
{

bool GridRGBDModeler :: addNewView(const RGBDImage& image, Pose3D& depth_pose)
{
    Pose3D rgb_pose = depth_pose;
    rgb_pose.toRightCamera(image.calibration()->rgb_intrinsics, image.calibration()->R, image.calibration()->T);

    Pose3D world_to_camera_normal_pose;
    world_to_camera_normal_pose.applyTransformBefore(Vec3f(0,0,0), depth_pose.cvEulerRotation());
    Pose3D camera_to_world_normal_pose = world_to_camera_normal_pose; camera_to_world_normal_pose.invert();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    rgbdImageToPointCloud(*cloud, image, depth_pose);
    *m_current_cloud += *cloud;

    pcl::ApproximateVoxelGrid< pcl::PointXYZ > filter;
    filter.setLeafSize(m_resolution, m_resolution, m_resolution);
    filter.setInputCloud(m_current_cloud);
    pcl::PointCloud<pcl::PointXYZ> subsampled_cloud;
    filter.filter(subsampled_cloud);
    *m_current_cloud = subsampled_cloud;

    pointCloudToMesh(m_mesh, *m_current_cloud);
    m_mesh.colors.resize(m_mesh.vertices.size(), cv::Vec3b(255,255,255));

    if (m_colorize)
    {
        Pose3D rgb_pose = depth_pose;
        rgb_pose.toRightCamera(image.calibration()->rgb_intrinsics, image.calibration()->R, image.calibration()->T);

        foreach_idx(i, m_mesh.vertices)
        {
            Point3f p_rgb = rgb_pose.projectToImage(m_mesh.vertices[i]);
            if (!is_yx_in_range(image.rgb(), p_rgb.y, p_rgb.x))
                continue;

            cv::Vec3b color = bgr_to_rgb(image.rgb()(p_rgb.y, p_rgb.x));
            m_mesh.colors[i] = color;
        }
    }

    return true;
}

void GridRGBDModeler::reset()
{
    m_current_cloud->clear();
    RGBDModeler::reset();
}

} // ntk


