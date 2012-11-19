#include <ntk/utils/arg.h>
#include <ntk/utils/debug.h>
#include <ntk/camera/rgbd_processor.h>
#include <ntk/mesh/mesh.h>
#include <ntk/mesh/pcl_utils.h>
#include <ntk/mesh/mesh_generator.h>
#include <ntk/geometry/relative_pose_estimator_icp.h>
#include <ntk/geometry/relative_pose_estimator_rgbd_icp.h>
#include <ntk/geometry/relative_pose_estimator_from_image.h>

#include <pcl/filters/random_sample.h>

#include <QApplication>
#include <iostream>

namespace cmd {
ntk::arg<int> debug_level ("--debug", "Debug level", 1);
// ntk::arg<CString> input_mesh1  (0, "input mesh 1", "mesh1.ply");
// ntk::arg<CString> input_mesh2  (0, "input mesh 2", "mesh2.ply");
ntk::arg<const char*> input_image1  (0, "input image 1", 0);
ntk::arg<const char*> input_image2  (0, "input image 2", 0);
ntk::arg<const char*> calibration_file (0, "Calibration file", 0);
}

namespace global {
ntk::Mesh mesh1;
ntk::Mesh mesh2;
}

using namespace ntk;
using namespace pcl;

bool alignWithICP(const RGBDImage& image1, const RGBDImage& image2)
{
    typedef pcl::PointNormal PointT;

    pcl::PointCloud<PointT>::Ptr source_cloud; // cloud1
    source_cloud.reset(new pcl::PointCloud<PointT>());

    pcl::PointCloud<PointT>::Ptr sampled_source_cloud; // cloud1
    sampled_source_cloud.reset(new pcl::PointCloud<PointT>());

    pcl::PointCloud<PointT>::Ptr sampled_target_cloud; // cloud0
    sampled_target_cloud.reset(new pcl::PointCloud<PointT>());

    RelativePoseEstimatorICPWithNormals<pcl::PointNormal> pose_estimator_icp;
    // RelativePoseEstimatorICP<PointT> pose_estimator_icp;
    pose_estimator_icp.setVoxelSize(0.001);
    pose_estimator_icp.setDistanceThreshold(0.05);
    pose_estimator_icp.setRANSACOutlierRejectionThreshold(0.05);
    pose_estimator_icp.setMaxIterations(100);

    meshToPointCloud(*source_cloud, global::mesh2);
    meshToPointCloud(*sampled_target_cloud, global::mesh1);

    // *sampled_source_cloud = *source_cloud;
    NormalCloudSampler<PointT> sampler;
    sampler.subsample(*source_cloud, *sampled_source_cloud, 1000);

#if 0
    pcl::RandomSample<pcl::PointNormal> sampler;
    sampler.setSample(100000);
    sampler.setInputCloud(source_cloud);
    sampler.filter(*sampled_source_cloud);
#endif

    pose_estimator_icp.setSourceCloud(sampled_source_cloud);
    pose_estimator_icp.setTargetCloud(sampled_target_cloud);

    bool ok = pose_estimator_icp.estimateNewPose();
    if (!ok)
        return false;

    Mesh mesh = global::mesh2;
    mesh.applyTransform(pose_estimator_icp.estimatedSourcePose().inverted());
    mesh.saveToPlyFile("aligned_icp.ply");
    return true;
}

bool alignWithLevenbergICP(const RGBDImage& image1, const RGBDImage& image2)
{
    typedef pcl::PointNormal PointT;

    pcl::PointCloud<PointT>::Ptr source_cloud; // cloud1
    source_cloud.reset(new pcl::PointCloud<PointT>());

    pcl::PointCloud<PointT>::Ptr sampled_source_cloud; // cloud1
    sampled_source_cloud.reset(new pcl::PointCloud<PointT>());

    pcl::PointCloud<PointT>::Ptr sampled_target_cloud; // cloud0
    sampled_target_cloud.reset(new pcl::PointCloud<PointT>());

    RelativePoseEstimatorRGBDICP<pcl::PointNormal> pose_estimator_icp;
    // RelativePoseEstimatorICP<PointT> pose_estimator_icp;
    pose_estimator_icp.setVoxelSize(0.001);
    pose_estimator_icp.setDistanceThreshold(0.05);
    pose_estimator_icp.setRANSACOutlierRejectionThreshold(0.05);
    pose_estimator_icp.setMaxIterations(100);

    meshToPointCloud(*source_cloud, global::mesh2);
    meshToPointCloud(*sampled_target_cloud, global::mesh1);

    // *sampled_source_cloud = *source_cloud;
    NormalCloudSampler<PointT> sampler;
    sampler.subsample(*source_cloud, *sampled_source_cloud, 1000);

    pose_estimator_icp.setSourceCloud(sampled_source_cloud);
    pose_estimator_icp.setTargetCloud(sampled_target_cloud);

    bool ok = pose_estimator_icp.estimateNewPose();
    if (!ok)
        return false;

    Mesh mesh = global::mesh2;
    mesh.applyTransform(pose_estimator_icp.estimatedSourcePose().inverted());
    mesh.saveToPlyFile("aligned_levenberg_icp.ply");
    return true;
}

bool alignWithRGBDICP(const RGBDImage& image1, const RGBDImage& image2)
{
    FeatureSetParams params ("FAST", "BRIEF64", true);
    RelativePoseEstimatorFromRgbFeatures pose_estimator (params);
    pose_estimator.setPostProcessWithRGBDICP(true);

    pose_estimator.setSourceImage(image2);
    pose_estimator.setTargetImage(image1);

    bool ok = pose_estimator.estimateNewPose();
    if (!ok)
        return false;

    Mesh mesh = global::mesh2;
    mesh.applyTransform(pose_estimator.estimatedSourcePose().inverted());
    mesh.saveToPlyFile("aligned_rgbd_icp.ply");
    return true;
}

bool alignWithRGBDThenICP(const RGBDImage& image1, const RGBDImage& image2)
{
    FeatureSetParams params ("FAST", "BRIEF64", true);
    RelativePoseEstimatorFromRgbFeatures pose_estimator (params);
    pose_estimator.setPostProcessWithRGBDICP(false);

    pose_estimator.setSourceImage(image2);
    pose_estimator.setTargetImage(image1);

    bool ok = pose_estimator.estimateNewPose();
    if (!ok)
        return false;

    typedef pcl::PointNormal PointT;

    pcl::PointCloud<PointT>::Ptr source_cloud; // cloud1
    source_cloud.reset(new pcl::PointCloud<PointT>());

    pcl::PointCloud<PointT>::Ptr sampled_source_cloud; // cloud1
    sampled_source_cloud.reset(new pcl::PointCloud<PointT>());

    pcl::PointCloud<PointT>::Ptr sampled_target_cloud; // cloud0
    sampled_target_cloud.reset(new pcl::PointCloud<PointT>());

    RelativePoseEstimatorICPWithNormals<pcl::PointNormal> pose_estimator_icp;
    // RelativePoseEstimatorICP<PointT> pose_estimator_icp;
    pose_estimator_icp.setVoxelSize(0.001);
    pose_estimator_icp.setDistanceThreshold(0.05);
    pose_estimator_icp.setRANSACOutlierRejectionThreshold(0.02);
    pose_estimator_icp.setMaxIterations(100);

    meshToPointCloud(*source_cloud, global::mesh2);
    meshToPointCloud(*sampled_target_cloud, global::mesh1);

    // *sampled_source_cloud = *source_cloud;
    // CloudSampler<PointT> sampler;
    NormalCloudSampler<PointT> sampler;
    sampler.subsample(*source_cloud, *sampled_source_cloud, 1000);

    pose_estimator_icp.setSourceCloud(sampled_source_cloud);
    pose_estimator_icp.setTargetCloud(sampled_target_cloud);
    pose_estimator_icp.setInitialSourcePoseEstimate(pose_estimator.estimatedSourcePose());

    ok = pose_estimator_icp.estimateNewPose();
    if (!ok)
        return false;

    Mesh mesh = global::mesh2;
    mesh.applyTransform(pose_estimator_icp.estimatedSourcePose().inverted());
    mesh.saveToPlyFile("aligned_rgbd_then_icp.ply");
    return true;
}

bool alignWithRGBD(const RGBDImage& image1, const RGBDImage& image2)
{
    FeatureSetParams params ("FAST", "BRIEF64", true);
    RelativePoseEstimatorFromRgbFeatures pose_estimator (params);
    pose_estimator.setPostProcessWithRGBDICP(false);

    pose_estimator.setSourceImage(image2);
    pose_estimator.setTargetImage(image1);

    bool ok = pose_estimator.estimateNewPose();
    if (!ok)
        return false;

    Mesh mesh = global::mesh2;
    mesh.applyTransform(pose_estimator.estimatedSourcePose().inverted());
    mesh.saveToPlyFile("aligned_rgbd.ply");
    return true;
}

int
main (int argc, char* argv [])
{
    ntk::arg_base::set_help_option("-h");
    ntk::arg_parse(argc, argv);
    ntk::ntk_debug_level = cmd::debug_level();
    cv::setBreakOnError(true);

    QApplication app (argc, argv);

    pcl::console::setVerbosityLevel (pcl::console::L_DEBUG);

    RGBDCalibration calibration;
    calibration.loadFromFile(cmd::calibration_file());

    OpenniRGBDProcessor processor;

    RGBDImage image1 (cmd::input_image1(), &calibration, &processor);
    RGBDImage image2 (cmd::input_image2(), &calibration, &processor);;

    MeshGenerator generator;
    generator.setMeshType(MeshGenerator::TriangleMesh);
    generator.setUseColor(true);

    generator.generate(image1);
    global::mesh1 = generator.mesh();
    global::mesh1.computeNormalsFromFaces();

    generator.generate(image2);
    global::mesh2 = generator.mesh();
    global::mesh2.computeNormalsFromFaces();

    ntk_dbg(0) << "============ ALIGN WITH ICP ============";
    alignWithICP(image1, image2);

    ntk_dbg(0) << "============ ALIGN WITH LEVENBERG ICP ============";
    alignWithLevenbergICP(image1, image2);

    ntk_dbg(0) << "============ ALIGN WITH RGBD ============";
    alignWithRGBD(image1, image2);

    ntk_dbg(0) << "============ ALIGN WITH RGBD THEN ICP ============";
    alignWithRGBDThenICP(image1, image2);

    ntk_dbg(0) << "============ ALIGN WITH RGBD-ICP ============";
    alignWithRGBDICP(image1, image2);

    global::mesh1.normals.clear();
    global::mesh1.saveToPlyFile("debug_mesh1.ply");

    global::mesh2.normals.clear();
    global::mesh2.saveToPlyFile("debug_mesh2.ply");

    return 0;
}
