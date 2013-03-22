/**
 * Copyright (C) 2013 ManCTL SARL <contact@manctl.com>
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
 * Author: Nicolas Burrus <nicolas.burrus@manctl.com>
 */

#ifndef NTK_GEOMETRY_PLANE_ESTIMATOR_H
#define NTK_GEOMETRY_PLANE_ESTIMATOR_H

// FIXME: For qtcreator, comment this out.
// #define NESTK_USE_PCL 1

#include <ntk/core.h>
#include <ntk/camera/rgbd_image.h>
#include <ntk/mesh/mesh.h>
#include <ntk/geometry/plane.h>
#include <ntk/numeric/differential_evolution_solver.h>

namespace ntk
{

  class PlaneSolver : public ntk::DifferentialEvolutionSolver
  {
  public:
    PlaneSolver(int dim,int pop)
      : ntk::DifferentialEvolutionSolver(dim,pop)
    {}

    virtual double EnergyFunction(double trial[],bool &bAtSolution);

    std::vector<cv::Point3f>& planePointsRef() { return g; }

  private:
    std::vector<cv::Point3f> g;
  };

  class PlaneEstimator
  {
    static const int dim = 4;
    static const int population = 30;
    static const int max_generations = 50;

  public:
    PlaneEstimator();
    void estimate(ntk::RGBDImage& image, cv::Mat1b& plane_points);
    const ntk::Plane& currentPlane() const { return m_plane; }
    const ntk::PlaneSolver& currentPlaneSolver();
    void getStructures(cv::Mat1b img, std::vector< std::vector<cv::Point> > contours);


  private:
    PlaneSolver m_solver;
    ntk::Plane m_plane;
    cv::Vec3f m_ref_plane;
  };

  class PclPlaneEstimator
  {
  public:
    void estimate(const ntk::RGBDImage& image, cv::Mat1b& plane_points, const Pose3D& pose);
    const ntk::Plane& currentPlane() const { return m_plane; }

  private:
    ntk::Plane m_plane;
  };

}

#endif // NTK_GEOMETRY_PLANE_ESTIMATOR_H
