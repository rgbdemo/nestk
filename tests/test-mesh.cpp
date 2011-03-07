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

#include <ntk/core.h>
#include <ntk/utils/debug.h>
#include <ntk/mesh/mesh.h>
#include <ntk/mesh/mesh_renderer.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/utils/time.h>

#include <opencv/highgui.h>

#include <QApplication>

using namespace ntk;
using namespace cv;

void test_renderer(Mesh& mesh)
{
  MeshRenderer renderer(800, 600);
  renderer.setMesh(mesh);
  Pose3D pose;
  pose.parseAvsFile("mesh_pose.avs");

  cv::Mat4b color_image (Size(800,600));
  cv::Mat1f depth;

  TimeCount tc_render("Rendering 100 times", 1);
  for (int i = 0; i < 100; ++i)
  {
    pose.applyTransformAfter(cv::Vec3f(0.02,0,0), cv::Vec3f(0,0,0));
    TimeCount tc_render_one("Rendering", 1);
    renderer.setMesh(mesh);
    renderer.setPose(pose, 0.3, 5);
    renderer.renderToImage(color_image, 0);
    tc_render_one.stop();
  }
  tc_render.stop();

  // color_image = renderer.colorBuffer();
  imwrite("debug_color.png", color_image);
  depth = renderer.depthBuffer();
  normalize(depth, depth, 0, 255, cv::NORM_MINMAX);
  imwrite("debug_depth.png", Mat1b(depth));
}

int main(int argc, char** argv)
{
  QApplication app(argc,argv); // renderer uses QT
  ntk::ntk_debug_level = 1;
  Mesh mesh;
  mesh.loadFromPlyFile("mesh.ply");
  mesh.saveToPlyFile("debug_output.ply");

  ntk_dbg_print(mesh.vertices.size(), 0);
  ntk_dbg_print(mesh.faces.size(), 0);
  ntk_dbg_print(mesh.colors.size(), 0);

  test_renderer(mesh);
  return 0;
}
