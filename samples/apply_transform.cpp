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
#include <ntk/camera/calibration.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/mesh/mesh.h>
#include <fstream>

#include <QDir>
#include <QDebug>

using namespace ntk;
using namespace cv;

// example command line (for copy-n-paste):
// calibrate_one_camera -w 8 -h 6 -o camera.yml images

namespace global
{
ntk::arg<const char*> opt_pose(0, "Pose to apply (.avs)", 0);
ntk::arg<const char*> opt_mesh(0, "Ply mesh to transform", 0);
ntk::arg<const char*> opt_images(0, "RGBD images directory to transform", 0);
ntk::arg<bool> opt_mesh_aligned("--mesh-already-aligned", "Whether the mesh was already aligned", false);

QDir images_dir;
QStringList images_list;
}

int main(int argc, char** argv)
{
    arg_base::set_help_option("--help");
    arg_parse(argc, argv);
    ntk::ntk_debug_level = 1;

    ntk::Pose3D pose;
    pose.parseAvsFile(global::opt_pose());
    ntk_ensure(pose.isValid(), "Could not read pose");

    std::string mesh_filename (global::opt_mesh());
    ntk::Mesh mesh;
    mesh.loadFromPlyFile(mesh_filename.c_str());

    if (!global::opt_mesh_aligned())
        mesh.applyTransform(pose);

    ntk::Rect3f bbox = ntk::bounding_box(mesh.vertices);

    Pose3D center_pose;
    center_pose.applyTransformBefore(-cv::Vec3f(bbox.x, 0, bbox.z), cv::Vec3f(0,0,0));
    pose.applyTransformAfter(center_pose);

    mesh.applyTransform(center_pose);
    mesh.saveToPlyFile(global::opt_mesh());

    pose.invert();

    global::images_dir = QDir(global::opt_images());
    ntk_ensure(global::images_dir.exists(), (global::images_dir.absolutePath() + " is not a directory.").toAscii());
    global::images_list = global::images_dir.entryList(QStringList("view????*"), QDir::Dirs, QDir::Name);

    ntk_ensure(global::images_list.size() > 0, "No images found.");

    std::vector<ntk::RGBDImage> images;
    loadImageList(global::images_dir, global::images_list, 0, 0, images);

    foreach_idx(i, images)
    {
        Pose3D new_pose;
        new_pose.parseAvsFile((images[i].directory() + "/rgb_pose.avs").c_str());
        if (!new_pose.isValid())
        {
            ntk_dbg(0) << "Warning: " << (images[i].directory() + "/rgb_pose.avs") << " is invalid";
            continue;
        }

        new_pose.applyTransformBefore(pose);
        new_pose.saveToAvsFile((images[i].directory() + "/rgb_pose.avs").c_str());
        ntk_dbg(0) << "[Stored] " << (images[i].directory() + "/rgb_pose.avs");
    }

    return 0;
}

