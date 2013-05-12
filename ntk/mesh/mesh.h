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

#ifndef NTK_MESH_MESH_H
# define NTK_MESH_MESH_H

#include <ntk/core.h>

#include <ntk/utils/opencv_utils.h>
#include <ntk/thread/event.h>

#include <vector>
#include <set>

namespace ntk
{

  class Pose3D;
  class Plane;

  struct Patch
  {
      int label;
      std::vector<int> inner_faces; // inside faces
      std::vector<int> border_faces; // inside faces with a border
      std::vector<int> outer_faces; // outside faces with a border
      std::set<int> inner_vertices; // vertices inside the patch
      std::set<int> border_vertices; // vertices inside the patch but on the frontier
      std::set<int> outer_vertices; // vertices outside but on the frontier
  };

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
    { indices[0] = i1; indices[1] = i2; indices[2] = i3; }

    Face() {}

    bool isValid() const { return indices[0] >= 0 && indices[1] >= 0 && indices[2] >= 0; }
    void kill() { indices[0] = -1; }

    bool operator==(const Face& rhs) const
    { return indices[0] == rhs.indices[0] && indices[1] == rhs.indices[1] && indices[2] == rhs.indices[2]; }

    void sort();

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
      FaceTexcoord() { std::fill(&u[0], &u[0] + 6, 0.0f); }

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
    explicit Mesh ();
    virtual ~Mesh ();

  public:
    Mesh (const Mesh& copy);
    Mesh& operator = (const Mesh& rhs);
    Mesh& swap (Mesh& other);

  public:
    std::vector<cv::Point3f> vertices;
    std::vector<cv::Vec3b> colors;
    std::vector<cv::Point3f> normals;
    std::vector<cv::Point2f> texcoords;
    std::vector<FaceTexcoord> face_texcoords;
    std::vector<Face> faces;
    std::vector<int> face_labels;
    cv::Mat3b texture;

  public:
    void loadFromPlyFile(const char* filename);
    void saveToPlyFile(const char* filename, bool use_binary = false) const;
    void saveToAsciiPlyFile(const char* filename) const;
    void saveToBinaryPlyFile(const char* filename) const;

    void saveToObjFile(const char* filename) const;
    void saveToStlFile(const char* filename) const;

    cv::Point3f centerize();
    cv::Point3f center() const;
    cv::Vec3f getFaceNormal(int face_i) const;

    void addPlane(const cv::Point3f& center, const cv::Point3f& normal, const cv::Point3f& sizes);
    void addCube(const cv::Point3f& center, const cv::Point3f& sizes, const cv::Vec3b& color = cv::Vec3b(255,0,0));
    void addSurfel(const Surfel& surfel);
    void addPointFromSurfel(const Surfel& surfel);
    void addMesh(const ntk::Mesh& rhs);

    void applyTransform(const Pose3D& pose);
    void applyScaleTransform(float x_scale, float y_scale, float z_scale);
    void computeNormalsFromFaces();
    void invertFaceNormals();
    void duplicateSharedVertices();
    void computeVertexFaceMap(std::vector< std::vector<int> >& faces_per_vertex) const;
    void computeFaceNeighbors(std::vector< std::vector<int> >& faces_neighbors,
                              const std::vector< std::vector<int> >& faces_per_vertex) const;
    void computeEdges(std::vector< Edge >& edges,
                      std::vector< std::vector<int> >& edges_per_vertex,
                      const std::vector< std::vector<int> >& faces_per_vertex) const;
    void extractConnectedComponents(std::vector<Patch>& components,
                                    const std::vector< std::vector<int> >& faces_neighbors) const;

    float computeLength(const Edge& edge) const;

    // Cleaning
  public:
    void removeDuplicatedVertices(float max_dist = 1e-10);
    void removeIsolatedVertices();
    void removeDeadFaces();
    void removeDuplicatedFaces();
    void removeNanVertices();

    /*!
     * Make sure no edge has more than 2 adjacent faces.
     * If it happens, remove the smallest faces.
     */
    void removeNonManifoldFaces();

    void removeFacesWithoutVisibility();

  public:
    bool hasColors() const { return colors.size() == vertices.size(); }
    bool hasNormals() const { return normals.size() == vertices.size(); }
    bool hasTexcoords() const { return texcoords.size() > 0; }
    bool hasFaceTexcoords() const { return face_texcoords.size() > 0; }
    bool hasFaces() const { return faces.size() > 0; }
    bool hasFaceLabels() const { return face_labels.size() > 0 && face_labels.size() == faces.size(); }

  public:
    float faceArea(int face_id) const;

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
