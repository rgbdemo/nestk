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

#ifndef NTK_MESH_MESH_H
# define NTK_MESH_MESH_H

#include <ntk/core.h>
#include <ntk/camera/calibration.h>
#include <ntk/geometry/plane.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/utils/opencv_utils.h>
#include <ntk/thread/event.h>
#include <vector>

namespace ntk
{

  struct Surfel
  {
    Surfel() : radius(0), confidence(0), n_views(0), min_camera_angle(0)
    {}

    bool enabled() const { return n_views > 0; }

    cv::Point3f location;
    cv::Point3f normal;
    cv::Vec3b color;
    float radius;
    float confidence;
    int n_views;
    float min_camera_angle;
    bool internal_seen_; // for exploration algorithms.
  };

  class Face
  {
  public:
    Face(unsigned i1, unsigned i2, unsigned i3)
    { indices[0] = i1; indices[1] = i2; indices[2] = i2; }

    Face() {}

    unsigned indices[3];
    static int numVertices() { return 3; };
  };

  class Mesh : public ntk::EventData
  {
  public:
    std::vector<cv::Point3f> vertices;
    std::vector<cv::Vec3b> colors;
    std::vector<cv::Point3f> normals;
    std::vector<cv::Point2f> texcoords;
    std::vector<Face> faces;
    cv::Mat3b texture;

  public:
    void loadFromPlyFile(const char* filename);
    void saveToPlyFile(const char* filename) const;
    cv::Point3f centerize();
    cv::Point3f center() const;
    void addCube(const cv::Point3f& center, const cv::Point3f& sizes, const cv::Vec3b& color = cv::Vec3b(255,0,0));
    void addSurfel(const Surfel& surfel);
    void addPointFromSurfel(const Surfel& surfel);
    void addMesh(const ntk::Mesh& rhs);
    void applyTransform(const Pose3D& pose);
    void applyScaleTransform(float x_scale, float y_scale, float z_scale);
    void computeNormalsFromFaces();

  public:
    bool hasColors() const { return colors.size() == vertices.size(); }
    bool hasNormals() const { return normals.size() == vertices.size(); }
    bool hasTexcoords() const { return texcoords.size() > 0; }
    bool hasFaces() const { return faces.size() > 0; }

  public:
    void clear();
  };
  ntk_ptr_typedefs(Mesh)

  void generate_mesh_from_plane(Mesh& mesh, const ntk::Plane& plane,
                                const cv::Point3f& center, float plane_size);

  void generate_mesh_from_cube(Mesh& mesh, const ntk::Rect3f& cube);


} // ntk

#endif // NTK_MESH_MESH_H
