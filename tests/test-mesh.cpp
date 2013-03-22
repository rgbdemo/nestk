
#include <ntk/core.h>
#include <ntk/utils/debug.h>
#include <ntk/mesh/mesh.h>
#include <ntk/mesh/mesh_renderer.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/utils/time.h>

#include <opencv2/highgui/highgui.hpp>

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
    pose.applyTransformAfter(cv::Vec3f(0.02f, 0, 0), cv::Vec3f(0, 0, 0));
    TimeCount tc_render_one("Rendering", 1);
    renderer.setMesh(mesh);
    float near_plane = 0.3f;
    float far_plane = 5.f;
    renderer.setPose(pose, &near_plane, &far_plane);
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
