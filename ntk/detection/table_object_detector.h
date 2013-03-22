/**
 * Copyright (C) 2013 ManCTL SARL <contact@manctl.com>
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
 * Author: Nicolas Burrus <nicolas.burrus@manctl.com>
 */

#ifndef NESTK_DETECTION_TABLE_OBJECT_DETECTOR_H
#define NESTK_DETECTION_TABLE_OBJECT_DETECTOR_H

#ifndef NESTK_USE_PCL
# error PCL support must be enabled to include this file.
#endif

#include <ntk/core.h>

#include <ntk/mesh/pcl_utils.h>
#include <ntk/geometry/plane.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

namespace ntk
{

/*!
 * This class is imported from PCL tutorials and detect clusters
 * lying on a flat table.
 */
template <class PointType>
class TableObjectDetector
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef PointType Point;
    typedef typename pcl::PointCloud<Point> PointCloud;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
#ifdef HAVE_PCL_GREATER_THAN_1_2_0
    typedef typename pcl::search::KdTree<pcl::PointXYZ>::Ptr KdTreePtr;
#else
    typedef typename pcl::KdTree<Point>::Ptr KdTreePtr;
#endif

public:
    TableObjectDetector ();
    void initialize();

public:
    float voxelSize() const { return downsample_leaf_objects_; }
    void setObjectVoxelSize(float s = 0.003) { downsample_leaf_objects_ = s; }
    void setBackgroundVoxelSize(float s = 0.01) { downsample_leaf_ = s; }
    void setDepthLimits(float min_z = -2, float max_z = -0.4) { min_z_bounds_ = min_z; max_z_bounds_ = max_z; }
    void setObjectHeightLimits(float min_h = 0.01, float max_h = 0.5) { object_min_height_ = min_h;  object_max_height_ = max_h; }
    void setMaxDistToPlane(float d) { m_max_dist_to_plane = d; }

public:
    /*! Returns true if at least one object and plane are detected. */
    bool detect(PointCloudConstPtr cloud);

    /*! Returns the index of the most central cluster. */
    int getMostCentralCluster() const;

public:
    const ntk::Plane& plane() const { return m_plane; }
    const std::vector <std::vector<cv::Point3f> >& objectClusters() const { return m_object_clusters; }
    PointCloudConstPtr tableInliers() const { return table_projected_; }

private:
    // PCL objects
    KdTreePtr normals_tree_, clusters_tree_;
    pcl::PassThrough<Point> pass_;
    pcl::VoxelGrid<Point> grid_, grid_objects_;
    pcl::NormalEstimation<Point, pcl::Normal> n3d_;
    pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;
    pcl::ProjectInliers<Point> proj_;
    pcl::ConvexHull<Point> hull_;
    pcl::ExtractPolygonalPrismData<Point> prism_;
    pcl::EuclideanClusterExtraction<Point> cluster_;

    double downsample_leaf_, downsample_leaf_objects_;
    int k_;
    double min_z_bounds_, max_z_bounds_;
    double sac_distance_threshold_;
    double normal_distance_weight_;

    // Min/Max height from the table plane object points will be considered from/to
    double object_min_height_, object_max_height_;

    // Object cluster tolerance and minimum cluster size
    double object_cluster_tolerance_, object_cluster_min_size_;

    // Maximal distance between the object and the plane.
    double m_max_dist_to_plane;

    // The raw, input point cloud data
    PointCloudConstPtr cloud_;
    // The filtered and downsampled point cloud data
    PointCloudConstPtr cloud_filtered_, cloud_downsampled_;
    // The resultant estimated point cloud normals for \a cloud_filtered_
    pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_;
    // The vector of indices from cloud_filtered_ that represent the planar table component
    pcl::PointIndices::ConstPtr table_inliers_;
    // The model coefficients of the planar table component
    pcl::ModelCoefficients::ConstPtr table_coefficients_;
    // The set of point inliers projected on the planar table component from \a cloud_filtered_
    PointCloudConstPtr table_projected_;
    // The convex hull of \a table_projected_
    PointCloudConstPtr table_hull_;
    // The remaining of the \a cloud_filtered_ which lies inside the \a table_hull_ polygon
    PointCloudConstPtr cloud_objects_, cloud_objects_downsampled_;

    ntk::Plane m_plane;
    std::vector< std::vector<cv::Point3f> > m_object_clusters;
};


} // ntk

#include "table_object_detector.hpp"

#endif // NESTK_DETECTION_TABLE_OBJECT_DETECTOR_H
