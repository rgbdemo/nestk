
#include <ntk/ntk.h>
#include <ntk/mesh/mesh_generator.h>
#include <ntk/utils/debug.h>
#include <ntk/camera/openni_grabber.h>
#include <ntk/mesh/pcl_utils.h>
#include <ntk/mesh/mesh_generator.h>
#include <ntk/geometry/relative_pose_estimator_icp.h>

#include <QApplication>
#include <QDir>
#include <QMutex>

using namespace cv;
using namespace ntk;

namespace opt
{
ntk::arg<int> kinect_id("--kinect-id", "Kinect id", 0);
}

int main(int argc, char **argv)
{
    // Parse command line options.
    arg_base::set_help_option("-h");
    arg_parse(argc, argv);

    // Set debug level to 1.
    ntk::ntk_debug_level = 1;

    // Set current directory to application directory.
    // This is to find Nite config in config/ directory.
    QApplication app (argc, argv);
    QDir::setCurrent(QApplication::applicationDirPath());

    // Declare the global OpenNI driver. Only one can be instantiated in a program.
    OpenniDriver ni_driver;

    // Declare the frame grabber.
    OpenniGrabber grabber(ni_driver, opt::kinect_id());

    // Start the grabber.
    grabber.connectToDevice();
    grabber.start();

    // Holder for the current image.
    RGBDImage ref_image, transformed_image;

    // Image post processor. Compute mappings when RGB resolution is 1280x1024.
    OpenniRGBDProcessor post_processor;

    // Wait for a new frame, get a local copy and postprocess it.
    grabber.waitForNextFrame();
    grabber.copyImageTo(ref_image);
    post_processor.processImage(ref_image);

    // Stop the grabber, don't need the kinect anymore.
    grabber.stop();
    grabber.disconnectFromDevice();

    // Artificially apply an in-plane rotation of 5 degrees.
    ref_image.copyTo(transformed_image);
    cv::warpAffine(ref_image.depth(),
                   transformed_image.depthRef(),
                   cv::getRotationMatrix2D(Point2f(320,240), 5, 1),
                   cv::Size(640,480),
                   cv::INTER_NEAREST);
    cv::warpAffine(ref_image.rgb(),
                   transformed_image.rgbRef(),
                   cv::getRotationMatrix2D(Point2f(320,240), 5, 1),
                   cv::Size(640,480),
                   cv::INTER_LINEAR);

    imshow_normalized("depth", ref_image.depth());
    imshow_normalized("rotated_depth", transformed_image.depth());
    cv::waitKey(0);

    Pose3D target_pose = *ref_image.calibration()->depth_pose;
    // Artificially apply some changes.
    // target_pose.applyTransformBefore(cv::Vec3f(0,0,0), cv::Vec3f(0,0,M_PI/2.0));
    target_pose.applyTransformBefore(cv::Vec3f(0,1.0f,0), cv::Vec3f(static_cast<double>(M_PI)*4.0f, 0, static_cast<float>(M_PI)/2.0f));

    // Check if ICP can find the applied transformation.
    RelativePoseEstimatorICP<pcl::PointXYZ> estimator;
    estimator.setDistanceThreshold(0.5); // points are associated if distance is < 50cm
    estimator.setVoxelSize(0.01); // points are clustered into 1cm cells.
    estimator.setMaxIterations(100); // no more than 100 ICP iterations
    estimator.setTargetImage(ref_image);
    estimator.setTargetPose(target_pose);
    estimator.setInitialSourcePoseEstimate(target_pose);
    estimator.setSourceImage(transformed_image);
    estimator.estimateNewPose();

    // Should be almost -0.0765679 0.996878 0.000200177
    ntk_dbg_print(estimator.estimatedSourcePose().cvTranslation(), 1);

    // Should be around -0.0329548 0.00738798 94.9662
    ntk_dbg_print(estimator.estimatedSourcePose().cvEulerRotation()*float(180/ntk::math::pi), 1);

    // Save the processed images as point clouds.
    MeshGenerator generator;
    generator.setUseColor(true);
    generator.setMeshType(MeshGenerator::PointCloudMesh);

    generator.generate(ref_image, target_pose, target_pose);
    generator.mesh().saveToPlyFile("ref.ply");

    generator.generate(transformed_image, target_pose, target_pose);
    generator.mesh().saveToPlyFile("transformed.ply");

    generator.generate(transformed_image, estimator.estimatedSourcePose(), estimator.estimatedSourcePose());
    generator.mesh().saveToPlyFile("transformed_corrected.ply");
}
