
#include <ntk/ntk.h>
#include <ntk/camera/openni_grabber.h>
#include <ntk/gesture/body_event.h>
#include <ntk/geometry/relative_pose_estimator_tabletop.h>
#include <ntk/mesh/mesh_generator.h>
#include <ntk/mesh/mesh_renderer.h>
#include <ntk/mesh/table_object_rgbd_modeler.h>
#include <ntk/detection/object/object_finder.h>
#include <ntk/detection/object/sift_object_match_lowe.h>
#include <ntk/utils/debug.h>

#include <QApplication>
#include <QDir>
#include <QMutex>

using cv::Point3f;
using namespace ntk;

namespace opt
{
ntk::arg<bool> high_resolution("--highres", "High resolution color image.", 0);
ntk::arg<int> kinect_id("--kinect-id", "Kinect id", 0);
ntk::arg<const char*> database(0, "Object database directory.", 0);
ntk::arg<float> max_depth("--maxdepth", "Maximal depth in meters", 2);
}

void show_detection_result(const char* window_name,
                           const RGBDImage& image,
                           const Mesh& source_mesh,
                           const Pose3D& pose,
                           const std::string& model_name)
{
    Mesh mesh = source_mesh;
    mesh.colors.resize(mesh.vertices.size());
    std::fill(mesh.colors.begin(), mesh.colors.end(), cv::Vec3b(255,0,0));
    // For debugging purpose, show the result.
    cv::Mat4b debug_img = toMat4b(image.rgb());
    MeshRenderer renderer(image.rgb().cols, image.rgb().rows);
    renderer.setMesh(mesh);
    renderer.setPose(pose);
    renderer.renderToImage(debug_img, MeshRenderer::WIREFRAME);
    cv::Mat3b final_img = toMat3b(debug_img);
    cv::putText(final_img, model_name, cv::Point(50, 50), 0, 1, cv::Scalar(255,0,0,255), 2);
    cv::imshow(window_name, final_img);
    while (((cv::waitKey(0)) & 0xff) != 27)
        ;
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

    // Initialize the object finder.
    ntk::ObjectFinderPtr object_finder;
    ObjectFinderParams params;
    params.detection_threshold = -15;
    params.feature = SiftParameters();
    params.object_database = opt::database();
    params.use_tracking = false;
    params.object_detector = "fpfh";
    params.keep_only_best_match = true;
    object_finder = ObjectFinderPtr(new ObjectFinder());
    object_finder->initialize(params);

    // Declare the global OpenNI driver. Only one can be instantiated in a program.
    OpenniDriver ni_driver;

    // Declare the frame grabber.
    OpenniGrabber grabber(ni_driver, opt::kinect_id());

    // High resolution 1280x1024 RGB Image.
    if (opt::high_resolution())
        grabber.setHighRgbResolution(true);

    // Start the grabber.
    grabber.connectToDevice();
    grabber.start();

    // Holder for the current image.
    RGBDImage image;

    // Image post processor. Compute mappings when RGB resolution is 1280x1024.
    OpenniRGBDProcessor post_processor;
    post_processor.setMaxDepth(opt::max_depth());
    post_processor.setFilterFlag(RGBDProcessorFlags::FilterThresholdDepth, true);

    // Wait for a new frame, get a local copy and postprocess it.
    grabber.waitForNextFrame();
    grabber.copyImageTo(image);
    post_processor.processImage(image);

    ntk_ensure(image.calibration(), "Uncalibrated rgbd image, cannot project to 3D!");

    // Detect the objects in the image.
    object_finder->processNewImage(image);
    int num_matches = object_finder->objectDetector()->nbObjectMatches();
    if (num_matches < 1)
    {
        ntk_dbg(1) << "No object found";
        return 1;
    }

    const ObjectMatch& match = object_finder->objectDetector()->objectMatch(0);
    ntk_dbg_print(match, 1);
    Pose3D object_pose = match.pose()->pose3d();
    Pose3D rgb_pose = *image.calibration()->rgb_pose;
    rgb_pose.applyTransformAfter(object_pose);

    ntk::Mesh mesh;
    match.model().loadMeshWithCache(mesh);

    show_detection_result("Initial pose mesh.", image, mesh, rgb_pose, match.model().name());

    // Pose is in rgb space, switch to depth space.
    Pose3D depth_pose = rgb_pose;
    depth_pose.toLeftCamera(image.calibration()->depth_intrinsics, image.calibration()->R, image.calibration()->T);

    RelativePoseEstimatorTableTop pose_estimator;
    pcl::PointCloud<pcl::PointXYZ> model_cloud;
    Mesh transformed_mesh = mesh;
    mesh.applyTransform(depth_pose);
    meshToPointCloud(model_cloud, transformed_mesh);

    pcl::PointCloud<pcl::PointXYZ> scene_cloud;
    vectorToPointCloud(scene_cloud, match.matchedPoints());

    pose_estimator.setTargetCloud(scene_cloud.makeShared());
    pose_estimator.setSourceCloud(model_cloud.makeShared());
    pose_estimator.estimateNewPose();
    Pose3D mesh_transform = pose_estimator.estimatedSourcePose();
    rgb_pose.applyTransformAfter(mesh_transform);

    show_detection_result("Final pose", image, mesh, rgb_pose, match.model().name());

    grabber.stop();
    grabber.disconnectFromDevice();
}
