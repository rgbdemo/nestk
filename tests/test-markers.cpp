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
#include <ntk/utils/debug.h>
#include <ntk/camera/openni_grabber.h>
#include <ntk/geometry/relative_pose_estimator_markers.h>
#include <ntk/mesh/mesh_generator.h>

#include <QDir>
#include <QApplication>

using namespace cv;
using namespace ntk;

namespace opt
{
ntk::arg<const char*> source_image(0, "Source image", 0);
ntk::arg<const char*> target_image(0, "Target image", 0);
ntk::arg<const char*> calibration_file(0, "Calibration file (yml)", 0);
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

    RGBDCalibration calibration;
    calibration.loadFromFile(opt::calibration_file());

    RGBDImage source_image, target_image;
    source_image.loadFromDir(opt::source_image(), &calibration);
    target_image.loadFromDir(opt::target_image(), &calibration);

    // Image post processor. Compute mappings when RGB resolution is 1280x1024.
    OpenniRGBDProcessor post_processor;

    post_processor.processImage(source_image);
    post_processor.processImage(target_image);

    Pose3D target_pose = *target_image.calibration()->depth_pose;

    RelativePoseEstimatorMarkers estimator;
    estimator.setSourceImage(source_image);
    estimator.setTargetImage(target_image);
    estimator.setTargetPose(target_pose);
    estimator.estimateNewPose();

    MeshGenerator generator;

    // Should be almost -0.0874827 0.997087 0.000867769
    ntk_dbg_print(estimator.estimatedSourcePose().cvTranslation(), 1);

    // Should be almost -0.350163 -0.0560269 95.1153
    ntk_dbg_print(estimator.estimatedSourcePose().cvEulerRotation()*float(180/ntk::math::pi), 1);

    // Save the processed images as point clouds.
    generator.generate(target_image, target_pose, target_pose);
    generator.mesh().saveToPlyFile("target.ply");

    generator.generate(source_image, target_pose, target_pose);
    generator.mesh().saveToPlyFile("source.ply");

    generator.generate(source_image, estimator.estimatedSourcePose(), estimator.estimatedSourcePose());
    generator.mesh().saveToPlyFile("source_transformed.ply");
}

