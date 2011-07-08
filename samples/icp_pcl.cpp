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

#include <ntk/ntk.h>
#include <ntk/mesh/mesh_generator.h>
#include <ntk/utils/debug.h>
#include <ntk/camera/openni_grabber.h>
#include <ntk/gesture/body_event.h>
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

    // Declare the frame grabber.
    OpenniGrabber grabber(opt::kinect_id());

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

    // Check if ICP can find the applied transformation.
    RelativePoseEstimatorICP estimator;
    estimator.setDistanceThreshold(0.5); // points are associated if distance is < 50cm
    estimator.setVoxelSize(0.01); // points are clustered into 1cm cells.
    estimator.setMaxIterations(100); // no more than 100 ICP iterations
    estimator.setReferenceImage(ref_image);
    estimator.estimateNewPose(transformed_image);

    // Should be almost 0
    ntk_dbg_print(estimator.currentPose().cvTranslation(), 1);

    // Should be around 5 degrees around the z axis.
    ntk_dbg_print(estimator.currentPose().cvEulerRotation()*float(180/ntk::math::pi), 1);

    // Save the processed images as point clouds.
    MeshGenerator generator;
    generator.setUseColor(true);
    generator.setMeshType(MeshGenerator::PointCloudMesh);

    generator.generate(ref_image);
    generator.mesh().saveToPlyFile("ref.ply");

    generator.generate(transformed_image);
    generator.mesh().saveToPlyFile("transformed.ply");

    generator.generate(transformed_image, estimator.currentPose(), estimator.currentPose());
    generator.mesh().saveToPlyFile("transformed_corrected.ply");
}
