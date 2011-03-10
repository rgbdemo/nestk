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

#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <sys/time.h>

using pcl::PointXYZ;

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
  mat << 0.000536227, -1.56178e-05, -9.47391e-05, -1.56178e-05, 0.000297322, -0.000148785, -9.47391e-05, -0.000148785, 9.7827e-05;
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
  test1();
  test2();
}
