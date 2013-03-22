
#include <ntk/core.h>
#include <ntk/utils/debug.h>
#include <ntk/mesh/mesh.h>
#include <ntk/mesh/pcl_utils.h>

#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>

#ifdef HAVE_PCL_GREATER_THAN_1_5_1
#include <pcl/surface/poisson.h>
#endif

using pcl::PointXYZ;

void testPoisson()
{
#ifdef HAVE_PCL_GREATER_THAN_1_5_1
    ntk::Mesh mesh;
    mesh.addCube(cv::Point3f(0,0,0), cv::Point3f(1,1,1));

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    cloud->width = 10;
    cloud->height = 1;
    cloud->points.resize(10);
    for (int i = 0; i < 10; ++i)
    {
        pcl::PointXYZRGBNormal p;
        p.x = i;
        p.y = i*2;
        p.z = i;
        p.normal_x = 1;
        p.normal_y = 0;
        p.normal_z = 0;
        p.r = 0;
        p.g = 0;
        p.b = 0;
        cloud->points[i] = p;
    }

    pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
    poisson.setInputCloud(cloud);
    pcl::PolygonMesh pcl_mesh;
    poisson.performReconstruction(pcl_mesh);

    polygonMeshToMesh(mesh, pcl_mesh);
    mesh.saveToPlyFile("poisson.ply");
#endif
}

void test1()
{
  PointXYZ p1, p2, p3;
  p1.x = 1; p1.y = p1.z = 0;
  p2.y = 1; p2.x = p2.z = 0;
  p3.z = 1; p3.x = p3.y = 0;
  double radius = getCircumcircleRadius (p1, p2, p3);
  ntk_dbg_print(radius, 0);
  // EXPECT_NEAR (radius, 0.816497, 1e-4);

  Eigen::Vector4f pt (1,0,0,0), line_pt (0,0,0,0), line_dir (1,1,0,0);
  double point2line_disance = sqrt (pcl::sqrPointToLineDistance (pt, line_pt, line_dir));
  // EXPECT_NEAR (point2line_disance, sqrt(2)/2, 1e-4);
}

void test2()
{
  Eigen::Matrix3f mat, vec;
  mat << 0.000536227f, -1.56178e-05f, -9.47391e-05f, -1.56178e-05f, 0.000297322f, -0.000148785f, -9.47391e-05f, -0.000148785f, 9.7827e-05f;
  Eigen::Vector3f val;

  pcl::eigen33 (mat, vec, val);

  //EXPECT_NEAR (fabs (vec (0, 0)), 0.168841, 1e-4); EXPECT_NEAR (fabs (vec (0, 1)), 0.161623, 1e-4); EXPECT_NEAR (fabs (vec (0, 2)), 0.972302, 1e-4);
  //EXPECT_NEAR (fabs (vec (1, 0)), 0.451632, 1e-4); EXPECT_NEAR (fabs (vec (1, 1)), 0.889498, 1e-4); EXPECT_NEAR (fabs (vec (1, 2)), 0.0694328, 1e-4);
  //EXPECT_NEAR (fabs (vec (2, 0)), 0.876082, 1e-4); EXPECT_NEAR (fabs (vec (2, 1)), 0.4274,   1e-4); EXPECT_NEAR (fabs (vec (2, 2)), 0.223178, 1e-4);

  //EXPECT_NEAR (val (0), 2.86806e-06, 1e-4); EXPECT_NEAR (val (1), 0.00037165, 1e-4); EXPECT_NEAR (val (2), 0.000556858, 1e-4);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig (mat);

  //EXPECT_NEAR (eig.eigenvectors () (0, 0), -0.168841, 1e-4); EXPECT_NEAR (eig.eigenvectors () (0, 1),  0.161623, 1e-4); EXPECT_NEAR (eig.eigenvectors () (0, 2),  0.972302, 1e-4);
  //EXPECT_NEAR (eig.eigenvectors () (1, 0), -0.451632, 1e-4); EXPECT_NEAR (eig.eigenvectors () (1, 1), -0.889498, 1e-4); EXPECT_NEAR (eig.eigenvectors () (1, 2),  0.0694328, 1e-4);
  //EXPECT_NEAR (eig.eigenvectors () (2, 0), -0.876083, 1e-4); EXPECT_NEAR (eig.eigenvectors () (2, 1),  0.4274,   1e-4); EXPECT_NEAR (eig.eigenvectors () (2, 2), -0.223178, 1e-4);

  //EXPECT_NEAR (eig.eigenvalues () (0), 2.86806e-06, 1e-4); EXPECT_NEAR (eig.eigenvalues () (1), 0.00037165, 1e-4); EXPECT_NEAR (eig.eigenvalues () (2), 0.000556858, 1e-4);

  Eigen::Vector3f eivals = mat.selfadjointView<Eigen::Lower>().eigenvalues ();

  //EXPECT_NEAR (eivals (0), 2.86806e-06, 1e-4); EXPECT_NEAR (eivals (1), 0.00037165, 1e-4); EXPECT_NEAR (eivals (2), 0.000556858, 1e-4);

}

int main (int argc, char** argv)
{
  // test1();
  // test2();
    testPoisson();
}
