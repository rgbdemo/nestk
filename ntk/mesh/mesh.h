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

    bool isValid() const { return indices[0] >= 0; }
    void kill() { indices[0] = -1; }

    int findIndexOf(int vertex) const
    {
        for (int k = 0; k < numVertices(); ++k)
            if (indices[k] == vertex)
                return k;
        return -1;
    }

    int indices[3];
    static int numVertices() { return 3; }
    static size_t size() { return 3; }
  };

  class Edge
  {
  public:
      Edge() : v1(-1) {}

      void setValue(int v1, int v2, int f1, int f2, float length)
      {
          if (v1 < v2)
          {
              this->v1 = v1;
              this->v2 = v2;
          }
          else
          {
              this->v1 = v2;
              this->v2 = v1;
          }
          this->f1 = f1;
          this->f2 = f2;
          this->length = length;
      }

      bool isValid() const { return v1 >= 0; }
      void kill() { v1 = -1; }

      // Vertex indices
      int v1;
      int v2;

      // Adjacent faces indices
      int f1;
      int f2;

      float length;

      bool hasVertex(int vertex) const
      {
          return v1 == vertex || v2 == vertex;
      }

      void updateFace(int old_index, int new_index)
      {
          if (f1 == old_index) f1 = new_index;
          if (f2 == old_index) f2 = new_index;
      }

      bool operator<(const Edge& rhs) const
      {
          if (v1 != rhs.v1) return v1 < rhs.v1;
          if (v2 != rhs.v2) return v2 < rhs.v2;
          if (f1 != rhs.f1) return f1 < rhs.f1;
          return f2 < rhs.f2;
      }
  };

  struct FaceTexcoord
  {
      FaceTexcoord() { std::fill(reinterpret_cast<float*>(this), reinterpret_cast<float*>(this)+6, 0.0f); }

      FaceTexcoord(const FaceTexcoord& rhs) { *this = rhs; }

      FaceTexcoord& operator= (const FaceTexcoord& rhs)
      {
          for (int i = 0; i < 3; ++i)
          {
              u[i] = rhs.u[i];
              v[i] = rhs.v[i];
          }
          return *this;
      }

      float u[3];
      float v[3];
  };

  class Mesh : public ntk::EventData
  {
    TYPEDEF_THIS(Mesh)

    CLONABLE_EVENT_DATA

  public:
    std::vector<cv::Point3f> vertices;
    std::vector<cv::Vec3b> colors;
    std::vector<cv::Point3f> normals;
    std::vector<cv::Point2f> texcoords;
    std::vector<FaceTexcoord> face_texcoords;
    std::vector<Face> faces;
    cv::Mat3b texture;

  public:
    void loadFromPlyFile(const char* filename);
    void saveToPlyFile(const char* filename) const;
    cv::Point3f centerize();
    cv::Point3f center() const;
    cv::Vec3f getFaceNormal(int face_i) const;
    void addCube(const cv::Point3f& center, const cv::Point3f& sizes, const cv::Vec3b& color = cv::Vec3b(255,0,0));
    void addSurfel(const Surfel& surfel);
    void addPointFromSurfel(const Surfel& surfel);
    void addMesh(const ntk::Mesh& rhs);
    void applyTransform(const Pose3D& pose);
    void applyScaleTransform(float x_scale, float y_scale, float z_scale);
    void computeNormalsFromFaces();
    void computeVertexFaceMap(std::vector< std::vector<int> >& faces_per_vertex) const;
    void computeFaceNeighbors(std::vector< std::vector<int> >& faces_neighbors,
                              const std::vector< std::vector<int> >& faces_per_vertex) const;
    void computeEdges(std::vector< Edge >& edges,
                      std::vector< std::vector<int> >& edges_per_vertex,
                      const std::vector< std::vector<int> >& faces_per_vertex) const;

    float computeLength(const Edge& edge) const;

    // Cleaning
  public:
    void removeDuplicatedVertices();
    void removeIsolatedVertices();

  public:
    bool hasColors() const { return colors.size() == vertices.size(); }
    bool hasNormals() const { return normals.size() == vertices.size(); }
    bool hasTexcoords() const { return texcoords.size() > 0; }
    bool hasFaceTexcoords() const { return face_texcoords.size() > 0; }
    bool hasFaces() const { return faces.size() > 0; }

  public:
    void clear();
  };
  ntk_ptr_typedefs(Mesh)

  void generate_mesh_from_plane(Mesh& mesh, const ntk::Plane& plane,
                                const cv::Point3f& center, float plane_size);

  void generate_mesh_from_cube(Mesh& mesh, const ntk::Rect3f& cube);

  cv::Point3f barycentricCoordinates(const cv::Point3f ref_points[3], const cv::Point3f& p);
  cv::Point3f fastBarycentricCoordinates(const cv::Point3f ref_points[3], const cv::Point3f& p);

} // ntk

#endif // NTK_MESH_MESH_H
