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

#include <pcl/filters/voxel_grid.h>

#include <QApplication>
#include <QDir>
#include <QMutex>

using namespace cv;
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

    // Declare the frame grabber.
    OpenniGrabber grabber(opt::kinect_id());

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

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>());
    rgbdImageToPointCloud(*cloud, image);
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setLeafSize (0.01, 0.01, 0.01); // leaf size is 1 cm
    grid.setFilterFieldName ("z");
    grid.setFilterLimits (-2.0, 0); // keep only data between 0 and 2 meters.
    grid.setInputCloud(cloud);
    grid.filter(*filtered_cloud);
    ntk_dbg_print(cloud->points.size(), 1);
    ntk_dbg_print(filtered_cloud->points.size(), 1);
    ntk::Mesh mesh;
    pointCloudToMesh(mesh, *filtered_cloud);

    ntk_dbg(0) << "Saving mesh file to output.ply...";
    mesh.saveToPlyFile("output.ply");
    grabber.stop();
}
