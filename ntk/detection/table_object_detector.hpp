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

# include "table_object_detector.h"
# include <ntk/utils/time.h>

# include <boost/make_shared.hpp>

using namespace pcl;
using namespace cv;

namespace ntk
{

template <class PointType>
TableObjectDetector<PointType> :: TableObjectDetector()
{
    // ---[ Create all PCL objects and set their parameters
    setObjectVoxelSize();
    setBackgroundVoxelSize();
    setDepthLimits();
    setObjectHeightLimits();

    // Normal estimation parameters
    k_ = 10; // 50 k-neighbors by default

    // Table model fitting parameters
    sac_distance_threshold_ = 0.01; // 1cm

    // Don't know ?
    normal_distance_weight_ = 0.1;

    // Clustering parameters
    object_cluster_tolerance_ = 0.05;        // 5cm between two objects
    object_cluster_min_size_  = 100;         // 100 points per object cluster
}

template <class PointType>
void TableObjectDetector<PointType> :: initialize()
{
    grid_.setLeafSize (downsample_leaf_, downsample_leaf_, downsample_leaf_);
    grid_objects_.setLeafSize (downsample_leaf_objects_, downsample_leaf_objects_, downsample_leaf_objects_);
    grid_.setFilterFieldName ("z");
    pass_.setFilterFieldName ("z");

    grid_.setFilterLimits (min_z_bounds_, max_z_bounds_);
    pass_.setFilterLimits (min_z_bounds_, max_z_bounds_);
    grid_.setDownsampleAllData (false);
    grid_objects_.setDownsampleAllData (false);

    normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
    clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
    clusters_tree_->setEpsilon (1);
    //tree_.setSearchWindowAsK (10);
    //tree_.setMaxDistance (0.5);

    n3d_.setKSearch (k_);
    n3d_.setSearchMethod (normals_tree_);

    seg_.setDistanceThreshold (sac_distance_threshold_);
    seg_.setMaxIterations (10000);

    normal_distance_weight_ = 0.1;

    seg_.setNormalDistanceWeight (normal_distance_weight_);
    seg_.setOptimizeCoefficients (true);
    seg_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg_.setMethodType (pcl::SAC_RANSAC);
    seg_.setProbability (0.99);

    proj_.setModelType (pcl::SACMODEL_NORMAL_PLANE);

    prism_.setHeightLimits (object_min_height_, object_max_height_);

    cluster_.setClusterTolerance (object_cluster_tolerance_);
    cluster_.setMinClusterSize (object_cluster_min_size_);
    cluster_.setSearchMethod (clusters_tree_);
}

template <class PointType>
bool TableObjectDetector<PointType> :: detect(pcl::PointCloud<Point>& cloud)
{
    ntk::TimeCount tc("TableObjectDetector::detect", 1);
    m_object_clusters.clear();
    initialize();

    ntk_dbg(1) << cv::format("PointCloud with %d data points.\n", cloud.width * cloud.height);

    // ---[ Convert the dataset
    cloud_ = cloud.makeShared();

    // ---[ Create the voxel grid
    pcl::PointCloud<Point> cloud_filtered;
    pass_.setInputCloud (cloud_);
    pass_.filter (cloud_filtered);
    cloud_filtered_.reset (new pcl::PointCloud<Point> (cloud_filtered));
    ntk_dbg(1) << cv::format("Number of points left after filtering (%f -> %f): %d out of %d.\n", min_z_bounds_, max_z_bounds_, (int)cloud_filtered.points.size (), (int)cloud.points.size ());

    pcl::PointCloud<Point> cloud_downsampled;
    grid_.setInputCloud (cloud_filtered_);
    grid_.filter (cloud_downsampled);
    cloud_downsampled_.reset (new pcl::PointCloud<Point> (cloud_downsampled));

    if ((int)cloud_filtered_->points.size () < k_)
    {
        ntk_dbg(0) << cv::format("WARNING Filtering returned %d points! Continuing.\n", (int)cloud_filtered_->points.size ());
        return false;
    }

    // ---[ Estimate the point normals
    pcl::PointCloud<pcl::Normal> cloud_normals;
    n3d_.setInputCloud (cloud_downsampled_);
    n3d_.compute (cloud_normals);
    cloud_normals_.reset (new pcl::PointCloud<pcl::Normal> (cloud_normals));
    ntk_dbg(1) << cv::format("%d normals estimated.", (int)cloud_normals.points.size ());

    //ROS_ASSERT (cloud_normals_->points.size () == cloud_filtered_->points.size ());

    // ---[ Perform segmentation
    pcl::PointIndices table_inliers; pcl::ModelCoefficients table_coefficients;
    seg_.setInputCloud (cloud_downsampled_);
    seg_.setInputNormals (cloud_normals_);
    seg_.segment (table_inliers, table_coefficients);
    table_inliers_.reset (new pcl::PointIndices (table_inliers));
    table_coefficients_.reset (new pcl::ModelCoefficients (table_coefficients));
    if (table_coefficients.values.size () > 3)
        ntk_dbg(1) << cv::format("Model found with %d inliers: [%f %f %f %f].\n", (int)table_inliers.indices.size (),
                                 table_coefficients.values[0], table_coefficients.values[1], table_coefficients.values[2], table_coefficients.values[3]);

    if (table_inliers_->indices.size () == 0)
        return false;

    m_plane = ntk::Plane (table_coefficients.values[0],
                          table_coefficients.values[1],
                          table_coefficients.values[2],
                          table_coefficients.values[3]);

    // ---[ Extract the table
    pcl::PointCloud<Point> table_projected;
    proj_.setInputCloud (cloud_downsampled_);
    proj_.setIndices (table_inliers_);
    proj_.setModelCoefficients (table_coefficients_);
    proj_.filter (table_projected);
    table_projected_.reset (new pcl::PointCloud<Point> (table_projected));
    ntk_dbg(1) << cv::format("Number of projected inliers: %d.\n", (int)table_projected.points.size ());

    // ---[ Estimate the convex hull
    pcl::PointCloud<Point> table_hull;
    hull_.setInputCloud (table_projected_);
    hull_.reconstruct (table_hull);
    table_hull_.reset (new pcl::PointCloud<Point> (table_hull));

    // ---[ Get the objects on top of the table
    pcl::PointIndices cloud_object_indices;
    prism_.setInputCloud (cloud_filtered_);
    prism_.setInputPlanarHull (table_hull_);
    prism_.segment (cloud_object_indices);
    ntk_dbg(1) << cv::format("Number of object point indices: %d.\n", (int)cloud_object_indices.indices.size ());

    pcl::PointCloud<Point> cloud_objects;
    pcl::ExtractIndices<Point> extract_object_indices;
    //extract_object_indices.setInputCloud (cloud_all_minus_table_ptr);
    extract_object_indices.setInputCloud (cloud_filtered_);
    //      extract_object_indices.setInputCloud (cloud_downsampled_);
    extract_object_indices.setIndices (boost::make_shared<const pcl::PointIndices> (cloud_object_indices));
    extract_object_indices.filter (cloud_objects);
    cloud_objects_.reset (new pcl::PointCloud<Point> (cloud_objects));
    ntk_dbg(1) << cv::format("Number of object point candidates: %d.\n", (int)cloud_objects.points.size ());

    if (cloud_objects.points.size () == 0)
        return false;

    // ---[ Downsample the points
    pcl::PointCloud<Point> cloud_objects_downsampled;
    grid_objects_.setInputCloud (cloud_objects_);
    grid_objects_.filter (cloud_objects_downsampled);
    cloud_objects_downsampled_.reset (new pcl::PointCloud<Point> (cloud_objects_downsampled));
    ntk_dbg(1) << cv::format("Number of object point candidates left after downsampling: %d.\n", (int)cloud_objects_downsampled.points.size ());

    // ---[ Split the objects into Euclidean clusters
    std::vector< PointIndices > object_clusters;
    cluster_.setInputCloud (cloud_objects_downsampled_);
    cluster_.extract (object_clusters);
    ntk_dbg(1) << cv::format("Number of clusters found matching the given constraints: %d.\n", (int)object_clusters.size ());

    for (size_t i = 0; i < object_clusters.size (); ++i)
    {
        std::vector<Point3f> object_points;
        foreach_idx(k, object_clusters[i].indices)
        {
            int index = object_clusters[i].indices[k];
            Point p = cloud_objects_downsampled_->points[index];
            object_points.push_back(Point3f(p.x,p.y,p.z));
        }
        m_object_clusters.push_back(object_points);
    }

    tc.stop();
    return true;
}

template <class PointType>
int TableObjectDetector<PointType> :: getMostCentralCluster() const
{
    // Look for the most central cluster which is not flying.

    // Closest object points must be at less than 2 cm from the plane.
    const float max_dist_to_plane_threshold = 0.02;

    int selected_object = -1;
    float min_x = FLT_MAX;
    for (int i = 0; i < objectClusters().size(); ++i)
    {
        const std::vector<Point3f>& object_points = objectClusters()[i];
        float min_dist_to_plane = FLT_MAX;
        for (int j = 0; j < object_points.size(); ++j)
        {
            Point3f pobj = object_points[j];
            min_dist_to_plane = std::min(plane().distanceToPlane(pobj), min_dist_to_plane);
        }

        if (min_dist_to_plane > max_dist_to_plane_threshold)
            continue;

        ntk::Rect3f bbox = bounding_box(object_points);
        if (std::abs(bbox.centroid().x) < min_x)
        {
            min_x = std::abs(bbox.centroid().x);
            selected_object = i;
        }
    }
    return selected_object;
}


} // ntk
