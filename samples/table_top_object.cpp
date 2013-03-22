
#include <ntk/ntk.h>
#include <ntk/mesh/mesh_generator.h>
#include <ntk/mesh/table_object_rgbd_modeler.h>
#include <ntk/utils/debug.h>
#include <ntk/camera/openni_grabber.h>

#include <QApplication>
#include <QDir>
#include <QMutex>

// disabled because of flann name conflict with PCL on Windows.
// using namespace cv;
using cv::Point3f;
using namespace ntk;

namespace opt
{
ntk::arg<bool> high_resolution("--highres", "High resolution color image.", 0);
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

    // Wait for a new frame, get a local copy and postprocess it.
    grabber.waitForNextFrame();
    grabber.copyImageTo(image);
    post_processor.processImage(image);

    ntk_ensure(image.calibration(), "Uncalibrated rgbd image, cannot project to 3D!");

    // Initialize the object modeler.
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    rgbdImageToPointCloud(*cloud, image);

    TableObjectDetector<PointXYZ> detector;
    detector.setObjectVoxelSize(0.003f); // 3 mm voxels.
    bool ok = detector.detect(cloud);
    ntk_throw_exception_if(!ok, "No cluster detected on a table plane!");

    for (int cluster_id = 0; cluster_id < detector.objectClusters().size(); ++cluster_id)
    {
        TableObjectRGBDModeler modeler;
        modeler.feedFromTableObjectDetector(detector, cluster_id);
        Pose3D pose = *image.calibration()->depth_pose;
        modeler.addNewView(image, pose);
        modeler.computeMesh();
        modeler.currentMesh().saveToPlyFile(cv::format("object%d.ply", cluster_id).c_str());
    }

    grabber.stop();
    grabber.disconnectFromDevice();
}
