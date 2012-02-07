#include "image_segmentor.h"

#include <ntk/detection/table_object_detector.h>

namespace ntk
{

bool ImageSegmentor ::
initializeFromFirstImage(const RGBDImage &image, const Pose3D &estimated_pose)
{
    normalized_pose = estimated_pose;
    delta_normalized_pose = normalized_pose;
    delta_normalized_pose.applyTransformBefore(image.calibration()->depth_pose->inverted());
    return true;
}

void ImageSegmentorFromObjectOnPlane ::
showCluster(const RGBDImage& image, const std::vector<cv::Point3f>& cluster)
{
    cv::Mat3b debug_img;
    image.rgb().copyTo(debug_img);
    foreach_idx(i, cluster)
    {
        cv::Point3f p_image = image.calibration()->rgb_pose->projectToImage(cluster[i]);
        if (is_yx_in_range(debug_img, p_image.y, p_image.x))
            debug_img(p_image.y, p_image.x) = cv::Vec3b(255,0,0);
    }
    cv::imwrite("debug_cluster.png", debug_img);
    // cv::imshow("Cluster", debug_img);
    // cv::waitKey(20);
}

bool ImageSegmentorFromObjectOnPlane::initializeFromFirstImage(const ntk::RGBDImage &image, const ntk::Pose3D &estimated_pose)
{
    // Compute the plane equation.
    TableObjectDetector<pcl::PointXYZ> plane_estimator;
    plane_estimator.setMaxDistToPlane(0.3);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    rgbdImageToPointCloud(*cloud, image, estimated_pose);
    plane_estimator.detect(cloud);
    ntk_assert(plane_estimator.objectClusters().size() > 0, "Could not find a table plane and cluster");

    int cluster_id = plane_estimator.getMostCentralCluster();
    ntk_assert(cluster_id >= 0, "Could not find a valid central cluster");
    const std::vector<cv::Point3f>& cluster = plane_estimator.objectClusters()[cluster_id];
    showCluster(image, cluster);

    table_plane = plane_estimator.plane();

    // Axis angle rotation to get a top view aligned with the plane normal
    cv::Vec3f rotation = ntk::compute_axis_angle_rotation(cv::Vec3f(0,0,-1),
                                                          -table_plane.normal());
    cv::Point3f centroid = computeCentroid(cluster);
    cv::Point3f plane_centroid = table_plane.intersectionWithLine(centroid, centroid + cv::Point3f(table_plane.normal()));
    this->delta_normalized_pose = *image.calibration()->depth_pose;
    const float camera_distance = 1.0f; // 1 meter from the plane.
    delta_normalized_pose.applyTransformAfterRodrigues(cv::Vec3f(0,0,0), rotation);
    cv::Vec3f translation = cv::Point3f(0,0,-camera_distance) - delta_normalized_pose.cameraTransform(plane_centroid);
    delta_normalized_pose.applyTransformAfter(translation, cv::Vec3f(0,0,0));
    ntk_dbg_print(delta_normalized_pose.cameraTransform(plane_centroid), 1);
    delta_normalized_pose.invert();

    std::vector<cv::Point3f> transformed_cluster (cluster.size());
    foreach_idx(i, transformed_cluster)
    {
        transformed_cluster[i] = delta_normalized_pose.invCameraTransform(cluster[i]);
    }
    bounding_box = ntk::bounding_box(transformed_cluster);
    return true;
}

ImageSegmentorFromBackgroundPlane :: ImageSegmentorFromBackgroundPlane()
    : distance_to_plane(2.0f)
{
    bounding_box.x = -1;
    bounding_box.y = -1;
    bounding_box.z = -distance_to_plane + 0.05; // 5cm from the plane
    bounding_box.width = 2;
    bounding_box.height = 2;
    bounding_box.depth = distance_to_plane;
}

bool ImageSegmentorFromBackgroundPlane::initializeFromFirstImage(const ntk::RGBDImage &image, const ntk::Pose3D &estimated_pose)
{
    // Compute the plane equation.
    TableObjectDetector<pcl::PointXYZ> plane_estimator;
    plane_estimator.setBackgroundVoxelSize(0.02);
    plane_estimator.setMaxDistToPlane(0.3);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    rgbdImageToPointCloud(*cloud, image, estimated_pose);
    plane_estimator.detect(cloud);
    table_plane = plane_estimator.plane();
    ntk_dbg_print(table_plane.normal(), 1);

    cv::Mat1f debug_im;
    image.depth().copyTo(debug_im);
    for (size_t i = 0; i < plane_estimator.tableInliers()->points.size(); ++i)
    {
        cv::Point3f p = toOpencv(plane_estimator.tableInliers()->points[i]);
        p = image.calibration()->depth_pose->projectToImage(p);
        debug_im(p.y, p.x) = 4.0f;
    }
    imwrite_normalized("debug_plane.png", debug_im);

    // Axis angle rotation to get a top view aligned with the plane normal
    cv::Vec3f rotation = ntk::compute_axis_angle_rotation(cv::Vec3f(0,0,-1),
                                                          -table_plane.normal());
    // Compute the intersection with the center of the image.
    cv::Point3f centroid (0,0,0);
    cv::Point3f plane_centroid = table_plane.intersectionWithLine(centroid, centroid + cv::Point3f(0,0,1));
    delta_normalized_pose.applyTransformAfterRodrigues(cv::Vec3f(0,0,0), rotation);
    cv::Vec3f translation = cv::Point3f(0,0,-distance_to_plane) - delta_normalized_pose.cameraTransform(plane_centroid);
    delta_normalized_pose.applyTransformAfter(translation, cv::Vec3f(0,0,0));
    ntk_dbg_print(delta_normalized_pose.cameraTransform(plane_centroid), 1);
    delta_normalized_pose.invert();
    this->normalized_pose = *image.calibration()->depth_pose;
    this->normalized_pose.applyTransformBefore(delta_normalized_pose);
    return true;
}

bool ImageSegmentorFromBackgroundPlane::filterImage(ntk::RGBDImage &image, const ntk::Pose3D &estimated_pose)
{
    cv::Mat1b& depth_mask_im = image.depthMaskRef();
    const cv::Mat1f& depth_im = image.depth();
    for_all_rc(depth_mask_im)
    {
        if (!depth_mask_im(r,c))
            continue;

        if (depth_im(r,c) < 1e-5)
            continue;

        cv::Point3f p = estimated_pose.unprojectFromImage(cv::Point2f(c, r), depth_im(r,c));
        if (!bounding_box.isPointInside(p))
            depth_mask_im(r,c) = 0;
    }
    return true;
}

ImageSegmentorFromMarkers::ImageSegmentorFromMarkers(const ntk::MarkerSetup& marker_setup)
    : marker_setup(marker_setup)
{
    bounding_box = ntk::Rect3f();

    for (size_t i = 0; i < marker_setup.markers.size(); ++i)
    {
        for (size_t j = 0; j < marker_setup.markers[i].corners().size(); ++j)
        {
            cv::Point3f p = marker_setup.markers[j].corners()[j];
            bounding_box.extendToInclude(p);
        }
    }

    bounding_box.y -= 0.05; // FIXME: foot is shifted from the board.
    bounding_box.height += 0.05; // FIXME: foot is shifted from the board.
    bounding_box.x -= 0.05;
    bounding_box.width += 0.1; // FIXME: foot is shifted from the board.
    bounding_box.z = 0.05; // 5cm from the plane to exclude the markerboard.
    bounding_box.depth = 0.3; // 30cm from the plane

    ntk_dbg_print(bounding_box, 1);

    table_plane = ntk::Plane(cv::Vec3f(0,0,1), cv::Point3f(0,0,0));
}

bool ImageSegmentorFromMarkers::initializeFromFirstImage(const ntk::RGBDImage &image, const ntk::Pose3D &estimated_pose)
{
    AbsolutePoseEstimatorMarkers estimator;
    estimator.setMarkerSetup(marker_setup);
    estimator.setInputImage(&image);
    bool ok = estimator.estimateNewPose();
    if (!ok)
        return false;
    normalized_pose = estimator.estimatedPose();
    delta_normalized_pose = normalized_pose;
    delta_normalized_pose.applyTransformBefore(image.calibration()->depth_pose->inverted());
    return true;
}

} // namespace ntk
