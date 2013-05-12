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


#include "mesh.h"
#include "ply.h"

#include <ntk/geometry/pose_3d.h>
#include <ntk/geometry/plane.h>
#include <ntk/utils/debug.h>
#include <ntk/utils/opencv_utils.h>
#include <ntk/numeric/utils.h>

// #include <opencv2/core/core.hpp>

#include <ntk/utils/time.h>

#include <fstream>
#include <cstring>
#include <errno.h>
#include <set>
#include <queue>
#include <utility>

using namespace cv;

namespace
{

struct PlyVertex
{
    float x,y,z;
    float nx,ny,nz;
    float u,v; // tex coords.
    unsigned char r,g,b;     /* vertex color */
};

struct PlyFace
{
    unsigned char nverts;    /* number of vertex indices in list */
    int *verts;              /* vertex index list */
    unsigned char ntexcoord; /* number of tex coords */
    float *texcoord;         /* texture coordinates */
    double nx,ny,nz;         /* normal vector */
};

/* list of the kinds of elements in the user's object */
const char *elem_names[] =
{
    "vertex", "face"
};

ply::PlyProperty available_vertex_properties[] = {
    {"x", Float32, Float32, offsetof(PlyVertex,x), 0, 0, 0, 0},
    {"y", Float32, Float32, offsetof(PlyVertex,y), 0, 0, 0, 0},
    {"z", Float32, Float32, offsetof(PlyVertex,z), 0, 0, 0, 0},
    {"nx", Float32, Float32, offsetof(PlyVertex,nx), 0, 0, 0, 0},
    {"ny", Float32, Float32, offsetof(PlyVertex,ny), 0, 0, 0, 0},
    {"nz", Float32, Float32, offsetof(PlyVertex,nz), 0, 0, 0, 0},
    {"s", Float32, Float32, offsetof(PlyVertex,u), 0, 0, 0, 0},
    {"t", Float32, Float32, offsetof(PlyVertex,v), 0, 0, 0, 0},
    {"red", Uint8, Uint8, offsetof(PlyVertex,r), 0, 0, 0, 0},
    {"green", Uint8, Uint8, offsetof(PlyVertex,g), 0, 0, 0, 0},
    {"blue", Uint8, Uint8, offsetof(PlyVertex,b), 0, 0, 0, 0},
};

ply::PlyProperty available_face_properties[] = { /* list of property information for a face */
    {"vertex_indices", PLY_Int32, PLY_Int32, offsetof(PlyFace,verts), 1, Uint8, Uint8, offsetof(PlyFace,nverts)},
    {"texcoord", Float32, Float32, offsetof(PlyFace,texcoord), 1, Uint8, Uint8, offsetof(PlyFace,ntexcoord)},
};



}

namespace ntk
{

Mesh::Mesh ()
{

}

Mesh::~Mesh ()
{

}

Mesh::Mesh (const Mesh& copy)
: vertices(      copy.vertices)
, colors(        copy.colors)
, normals(       copy.normals)
, texcoords(     copy.texcoords)
, face_texcoords(copy.face_texcoords)
, faces(         copy.faces)
, face_labels(   copy.face_labels)
, texture(       copy.texture)
{

}

Mesh&
Mesh::operator = (const Mesh& rhs)
{
    Mesh that(rhs);
    return this->swap(that);
}

Mesh&
Mesh::swap (Mesh& other)
{
    vertices      .swap(other.vertices);
    colors        .swap(other.colors);
    normals       .swap(other.normals);
    texcoords     .swap(other.texcoords);
    face_texcoords.swap(other.face_texcoords);
    faces         .swap(other.faces);
    face_labels   .swap(other.face_labels);
    std::swap(texture,  other.texture);

    return *this;
}

void Mesh::applyTransform(const Pose3D& pose)
{
    foreach_idx(i, vertices)
            vertices[i] = pose.cameraTransform(vertices[i]);

    ntk::Pose3D normal_pose;
    normal_pose.applyTransformBefore(cv::Vec3f(0.f,0.f,0.f), pose.cvEulerRotation());

    foreach_idx(i, normals)
            normals[i] = normal_pose.cameraTransform(normals[i]);
}

Point3f Mesh :: centerize()
{
    Point3f center(0,0,0);
    foreach_idx(i, vertices)
    {
        center += vertices[i];
    }
    center *= 1.0/vertices.size();

    foreach_idx(i, vertices)
    {
        vertices[i] -= center;
    }
    return center;
}

Point3f Mesh :: center() const
{
    Point3f center(0,0,0);
    foreach_idx(i, vertices)
    {
        center += vertices[i];
    }
    center *= 1.0/vertices.size();
    return center;
}

cv::Vec3f Mesh::getFaceNormal(int face_i) const
{
    const Face& face = faces[face_i];
    Vec3f v01 = vertices[face.indices[1]] - vertices[face.indices[0]];
    Vec3f v02 = vertices[face.indices[2]] - vertices[face.indices[0]];
    Vec3f n = v01.cross(v02);
    ntk::normalize(n);
    return n;
}

void Mesh::addPlane(const Point3f &center, const Point3f &normal, const Point3f &sizes)
{
    Point3f line1[2] = { Point3f(center.x-sizes.x, center.y-sizes.y, center.z-sizes.z),
                         Point3f(center.x-sizes.x, center.y+sizes.y, center.z-sizes.z) };
    Point3f line2[2] = { Point3f(center.x+sizes.x, center.y-sizes.y, center.z-sizes.z),
                         Point3f(center.x+sizes.x, center.y+sizes.y, center.z-sizes.z) };
    Point3f line3[2] = { Point3f(center.x-sizes.x, center.y-sizes.y, center.z+sizes.z),
                         Point3f(center.x-sizes.x, center.y+sizes.y, center.z+sizes.z) };
    Point3f line4[2] = { Point3f(center.x+sizes.x, center.y-sizes.y, center.z+sizes.z),
                         Point3f(center.x+sizes.x, center.y+sizes.y, center.z+sizes.z) };

    ntk::Plane plane (normal, center);

    Point3f plane_p1 = plane.intersectionWithLine(line1[0], line1[1]);
    Point3f plane_p2 = plane.intersectionWithLine(line2[0], line2[1]);
    Point3f plane_p3 = plane.intersectionWithLine(line3[0], line3[1]);
    Point3f plane_p4 = plane.intersectionWithLine(line4[0], line4[1]);

    vertices.push_back(plane_p1);
    vertices.push_back(plane_p2);
    vertices.push_back(plane_p3);
    vertices.push_back(plane_p4);

    {
        Face f;
        f.indices[0] = 0;
        f.indices[1] = 1;
        f.indices[2] = 2;
        faces.push_back(f);
    }

    {
        Face f;
        f.indices[0] = 2;
        f.indices[1] = 1;
        f.indices[2] = 3;
        faces.push_back(f);
    }
}

void Mesh::saveToAsciiPlyFile(const char* filename) const
{   
    if (texture.data)
    {
        std::string texture_filename = filename;
        if (texture_filename.size() > 3)
        {
            texture_filename.erase(texture_filename.size()-4, 4); // remove .ply
            texture_filename += ".png";
        }
        else
        {
            texture_filename += ".texture.png";
        }
        imwrite(texture_filename, texture);
    }

    std::ofstream ply_file (filename);
    ply_file << "ply\n";
    ply_file << "format ascii 1.0\n";
    ply_file << "element vertex " << vertices.size() << "\n";
    ply_file << "property float x\n";
    ply_file << "property float y\n";
    ply_file << "property float z\n";

    if (hasNormals())
    {
        ply_file << "property float nx\n";
        ply_file << "property float ny\n";
        ply_file << "property float nz\n";
    }

    if (hasTexcoords())
    {
        // put it twice, blender uses (s,t) and meshlab (u,v)
        ply_file << "property float s\n";
        ply_file << "property float t\n";
    }

    if (hasColors())
    {
        ply_file << "property uchar red\n";
        ply_file << "property uchar green\n";
        ply_file << "property uchar blue\n";
    }

    if (hasFaces())
    {
        ply_file << "element face " << faces.size() << "\n";
        ply_file << "property list uchar uint vertex_indices\n";
        // For meshlab wedges.
        if (hasTexcoords() || hasFaceTexcoords())
            ply_file << "property list uchar float texcoord\n";
    }


    ply_file << "end_header\n";

    foreach_idx(i, vertices)
    {
        ply_file << vertices[i].x << " " << vertices[i].y << " " << vertices[i].z;

        if (hasNormals())
            ply_file << " " << (ntk::math::isnan(normals[i].x) ? 0 : normals[i].x)
                     << " " << (ntk::math::isnan(normals[i].y) ? 0 : normals[i].y)
                     << " " << (ntk::math::isnan(normals[i].z) ? 0 : normals[i].z);

        if (hasTexcoords())
        {
            ply_file << " " << texcoords[i].x << " " << texcoords[i].y;
        }

        if (hasColors())
            ply_file << " " << (int)colors[i][0] << " " << (int)colors[i][1] << " " << (int)colors[i][2];

        ply_file << "\n";
    }

    if (hasFaces())
    {
        foreach_idx(i, faces)
        {
            ply_file << faces[i].numVertices();
            for (unsigned j = 0; j < faces[i].numVertices(); ++j)
                ply_file << " " << faces[i].indices[j];

            if (hasTexcoords() && !hasFaceTexcoords())
            {
                ply_file << "\n6";
                for (unsigned j = 0; j < faces[i].numVertices(); ++j)
                {
                    ply_file << " " << texcoords[faces[i].indices[j]].x;
                    ply_file << " " << 1.0 - texcoords[faces[i].indices[j]].y;
                }
            }
            else if (hasFaceTexcoords())
            {
                ply_file << "\n6";
                for (unsigned j = 0; j < faces[i].numVertices(); ++j)
                {
                    ply_file << " " << face_texcoords[i].u[j];
                    ply_file << " " << 1.0 - face_texcoords[i].v[j];
                }
            }
            ply_file << "\n";
        }
    }

    ply_file.close();
}

void Mesh::saveToPlyFile(const char *filename, bool use_binary) const
{
    if (use_binary)
        saveToBinaryPlyFile(filename);
    else
        saveToAsciiPlyFile(filename);
}

void Mesh::saveToBinaryPlyFile(const char *filename) const
{
    if (texture.data)
    {
        std::string texture_filename = filename;
        if (texture_filename.size() > 3)
        {
            texture_filename.erase(texture_filename.size()-4, 4); // remove .ply
            texture_filename += ".png";
        }
        else
        {
            texture_filename += ".texture.png";
        }
        imwrite(texture_filename, texture);
    }

    std::ofstream ply_file (filename, std::ios_base::out | std::ios_base::binary);
    ply_file << "ply\n";
    if (QSysInfo::ByteOrder == QSysInfo::LittleEndian)
        ply_file << "format binary_little_endian 1.0\n";
    else
        ply_file << "format binary_big_endian 1.0\n";
    ply_file << "element vertex " << vertices.size() << "\n";
    ply_file << "property float x\n";
    ply_file << "property float y\n";
    ply_file << "property float z\n";

    if (hasNormals())
    {
        ply_file << "property float nx\n";
        ply_file << "property float ny\n";
        ply_file << "property float nz\n";
    }

    if (hasTexcoords())
    {
        // put it twice, blender uses (s,t) and meshlab (u,v)
        ply_file << "property float s\n";
        ply_file << "property float t\n";
    }

    if (hasColors())
    {
        ply_file << "property uchar red\n";
        ply_file << "property uchar green\n";
        ply_file << "property uchar blue\n";
    }

    if (hasFaces())
    {
        ply_file << "element face " << faces.size() << "\n";
        ply_file << "property list uchar uint vertex_indices\n";
        // For meshlab wedges.
        if (hasTexcoords() || hasFaceTexcoords())
            ply_file << "property list uchar float texcoord\n";
    }


    ply_file << "end_header\n";

    foreach_idx(i, vertices)
    {
        ply_file.write(reinterpret_cast<const char*>(&vertices[i].x), sizeof(float)*3);

        if (hasNormals())
        {
            cv::Vec3f n = normals[i];
            for (int k = 0; k < 3; ++k)
                if (ntk::math::isnan(n[k]))
                    n[k] = 0.f;
            ply_file.write(reinterpret_cast<const char*>(&n[0]), sizeof(float)*3);
        }

        if (hasTexcoords())
        {
            ply_file.write(reinterpret_cast<const char*>(&texcoords[i].x), sizeof(float)*2);
        }

        if (hasColors())
            ply_file.write(reinterpret_cast<const char*>(&colors[i][0]), sizeof(uchar)*3);
    }

    if (hasFaces())
    {
        foreach_idx(i, faces)
        {
            uchar num_vertices = faces[i].numVertices();
            ply_file.write(reinterpret_cast<const char*>(&num_vertices), sizeof(uchar));
            ply_file.write(reinterpret_cast<const char*>(&faces[i].indices[0]), 3*sizeof(int));

            uchar num_texcoords = 6;
            if (hasTexcoords() && !hasFaceTexcoords())
            {
                ply_file.write(reinterpret_cast<const char*>(&num_texcoords), sizeof(uchar));
                for (unsigned j = 0; j < faces[i].numVertices(); ++j)
                {
                    cv::Point2f uv (texcoords[faces[i].indices[j]].x, 1.0 - texcoords[faces[i].indices[j]].y);
                    ply_file.write(reinterpret_cast<const char*>(&uv.x), 2*sizeof(float));
                }
            }
            else if (hasFaceTexcoords())
            {
                ply_file.write(reinterpret_cast<const char*>(&num_texcoords), sizeof(uchar));
                for (unsigned j = 0; j < faces[i].numVertices(); ++j)
                {
                    cv::Point2f uv (face_texcoords[i].u[j], 1.0 - face_texcoords[i].v[j]);
                    ply_file.write(reinterpret_cast<const char*>(&uv.x), 2*sizeof(float));
                }
            }
        }
    }

    ply_file.close();
}

void Mesh::saveToObjFile(const char *filename) const
{
    std::ofstream obj_file (filename);
    obj_file << "# Generated.\n";

    foreach_idx (i, vertices)
    {
        const cv::Point3f& v = vertices[i];
        obj_file << "v " << v.x << " " << v.y << " " << v.z;

        if (hasColors())
        {
            const cv::Vec3b& c = colors[i];
            obj_file << " " << c[0]/255.f << " " << c[1]/255.f << " " << c[2]/255.f;
        }

        obj_file << "\n";
    }

    obj_file << "\n";

    foreach_idx (i, faces)
    {
        const Face& face = faces[i];
        obj_file << "f " << face.indices[0]+1 << " " << face.indices[1]+1 << " " << face.indices[2]+1 << "\n";
    }

    obj_file << "# End of file.\n";
}

void Mesh::saveToStlFile(const char *filename) const
{
    std::ofstream stl_file (filename);
    stl_file << "solid \n";
    foreach_idx (i, faces)
    {
        cv::Vec3f n = getFaceNormal(i);
        stl_file << "facet normal " << n[0] << " " << n[1] << " " << n[2] << "\n";
        stl_file << "outer loop\n";
        for (int k = 0; k < 3; ++k)
        {
            const cv::Point3f& v = vertices[faces[i].indices[k]];
            stl_file << "vertex " << v.x << " " << v.y << " " << v.z << "\n";
        }
        stl_file << "endloop\n";
        stl_file << "endfacet\n";
    }
}

void Mesh::loadFromPlyFile(const char* filename)
{
    vertices.clear();
    colors.clear();
    texcoords.clear();
    normals.clear();
    faces.clear();

    bool has_colors = false;
    bool has_normals = false;
    bool has_texcoords = false;
    bool has_faces = false;

    std::vector<ply::PlyProperty> vertex_properties;
    std::vector<ply::PlyProperty> face_properties;

    std::vector<PlyVertex> ply_vertices;
    std::vector<PlyFace> ply_faces;

    FILE* mesh_file = fopen(filename, "rb");
    int err = errno;
    if (err)
    {
        ntk_dbg(0) << "[ERROR] " << strerror(err);
    }
    ntk_throw_exception_if(!mesh_file, "Could not open mesh file.");

    ply::PlyFile* ply_file = ply::read_ply(mesh_file);
    ntk_throw_exception_if(!ply_file, "Could not parse mesh file.");

    for (int i = 0; i < ply_file->num_elem_types; i++)
    {
        /* prepare to read the i'th list of elements */
        int elem_count = 0;
        const char* elem_name = ply::setup_element_read_ply (ply_file, i, &elem_count);

        if (ply::equal_strings("vertex", elem_name))
        {
            /* create a vertex list to hold all the vertices */
            ply_vertices.resize(elem_count);

            /* set up for getting vertex elements */
            ply::setup_property_ply (ply_file, &available_vertex_properties[0]);
            ply::setup_property_ply (ply_file, &available_vertex_properties[1]);
            ply::setup_property_ply (ply_file, &available_vertex_properties[2]);

            if (has_property(ply_file, "vertex", "nx"))
            {
                has_normals = true;
                ply::setup_property_ply (ply_file, &available_vertex_properties[3]);
                ply::setup_property_ply (ply_file, &available_vertex_properties[4]);
                ply::setup_property_ply (ply_file, &available_vertex_properties[5]);
            }

            if (has_property(ply_file, "vertex", "s"))
            {
                has_texcoords = true;
                ply::setup_property_ply (ply_file, &available_vertex_properties[6]);
                ply::setup_property_ply (ply_file, &available_vertex_properties[7]);
            }

            if (has_property(ply_file, "vertex", "red"))
            {
                has_colors = true;
                ply::setup_property_ply (ply_file, &available_vertex_properties[8]);
                ply::setup_property_ply (ply_file, &available_vertex_properties[9]);
                ply::setup_property_ply (ply_file, &available_vertex_properties[10]);
            }

            /* grab all the vertex elements */
            for (int j = 0; j < elem_count; j++)
                ply::get_element_ply (ply_file, &ply_vertices[j]);
        }
        else if (ply::equal_strings("face", elem_name))
        {
            has_faces = true;

            /* create a list to hold all the face elements */
            ply_faces.resize(elem_count);

            /* set up for getting face elements */
            setup_property_ply (ply_file, &available_face_properties[0]);

            if (has_property(ply_file, "face", "texcoord"))
            {
                setup_property_ply (ply_file, &available_face_properties[1]);
            }

            // setup_property_ply (ply_file, &global::face_props[1]);

            /* grab all the face elements */
            for (int j = 0; j < elem_count; j++)
            {
                get_element_ply (ply_file, &ply_faces[j]);
            }
        }
    }
    fclose(mesh_file);

    vertices.resize(ply_vertices.size());
    if (has_colors) colors.resize(vertices.size());
    if (has_normals) normals.resize(vertices.size());

    foreach_idx(i, ply_vertices)
    {
        const PlyVertex& v = ply_vertices[i];
        vertices[i] = Point3f(v.x, v.y, v.z);
        if (has_colors)
            colors[i] = Vec3b(v.r, v.g, v.b);
        if (has_normals)
            normals[i] = Point3f(v.nx, v.ny, v.nz);
    }

    if (has_faces)
    {
        faces.resize(ply_faces.size());
        foreach_idx(i, ply_faces)
        {
            PlyFace& f = ply_faces[i];
            ntk_ensure(f.nverts == 3, "Only triangles are supported.");
            for (int j = 0; j < f.nverts; ++j)
                faces[i].indices[j] = f.verts[j];
	    free (f.verts); // FIXME: should we free other fields?
        }
    }
}

void Mesh:: clear()
{
    vertices.clear();
    colors.clear();
    normals.clear();
    texcoords.clear();
    faces.clear();
    texture = cv::Mat3b();
}

void Mesh :: addPointFromSurfel(const Surfel& surfel)
{
    vertices.push_back(surfel.location);
    colors.push_back(surfel.color);
    normals.push_back(surfel.normal);
}

void Mesh :: addSurfel(const Surfel& surfel)
{
    int idx = vertices.size();

    ntk_assert(cv::norm(surfel.normal)>0.9, "Normal must be normalized and valid!");

    Vec3f v1, v2;
    orthogonal_basis(v1, v2, surfel.normal);
    Point3f p0 = surfel.location + Point3f(v1 * surfel.radius);
    Point3f p1 = surfel.location + Point3f(v1 * (surfel.radius/2.0f) + v2 * surfel.radius);
    Point3f p2 = surfel.location + Point3f(v1 * (-surfel.radius/2.0f) + v2 * surfel.radius);
    Point3f p3 = surfel.location + Point3f(v1 * -surfel.radius);
    Point3f p4 = surfel.location + Point3f(v1 * (-surfel.radius/2.0f) + v2 * (-surfel.radius));
    Point3f p5 = surfel.location + Point3f(v1 * (surfel.radius/2.0f) + v2 * (-surfel.radius));
    vertices.push_back(p0);
    vertices.push_back(p1);
    vertices.push_back(p2);
    vertices.push_back(p3);
    vertices.push_back(p4);
    vertices.push_back(p5);

    for (int k = 0; k < 6; ++k)
        colors.push_back(surfel.color);

    for (int k = 0; k < 6; ++k)
        normals.push_back(surfel.normal);

    {
        Face f;
        f.indices[0] = idx+5;
        f.indices[1] = idx+0;
        f.indices[2] = idx+1;
        faces.push_back(f);
    }

    {
        Face f;
        f.indices[0] = idx+5;
        f.indices[1] = idx+1;
        f.indices[2] = idx+2;
        faces.push_back(f);
    }

    {
        Face f;
        f.indices[0] = idx+4;
        f.indices[1] = idx+5;
        f.indices[2] = idx+2;
        faces.push_back(f);
    }

    {
        Face f;
        f.indices[0] = idx+4;
        f.indices[1] = idx+2;
        f.indices[2] = idx+3;
        faces.push_back(f);
    }
}


  void generate_mesh_from_plane(Mesh& mesh, const ntk::Plane& plane, const cv::Point3f& center, float plane_size)
  {
    Point3f line1[2] = { Point3f(center.x-plane_size, center.y-plane_size, center.z-plane_size),
                         Point3f(center.x-plane_size, center.y+plane_size, center.z-plane_size) };
    Point3f line2[2] = { Point3f(center.x+plane_size, center.y-plane_size, center.z-plane_size),
                         Point3f(center.x+plane_size, center.y+plane_size, center.z-plane_size) };
    Point3f line3[2] = { Point3f(center.x-plane_size, center.y-plane_size, center.z+plane_size),
                         Point3f(center.x-plane_size, center.y+plane_size, center.z+plane_size) };
    Point3f line4[2] = { Point3f(center.x+plane_size, center.y-plane_size, center.z+plane_size),
                         Point3f(center.x+plane_size, center.y+plane_size, center.z+plane_size) };

    Point3f plane_p1 = plane.intersectionWithLine(line1[0], line1[1]);
    Point3f plane_p2 = plane.intersectionWithLine(line2[0], line2[1]);
    Point3f plane_p3 = plane.intersectionWithLine(line3[0], line3[1]);
    Point3f plane_p4 = plane.intersectionWithLine(line4[0], line4[1]);

    mesh.vertices.push_back(plane_p1);
    mesh.vertices.push_back(plane_p2);
    mesh.vertices.push_back(plane_p3);
    mesh.vertices.push_back(plane_p4);

    {
        Face f;
        f.indices[0] = 0;
        f.indices[1] = 1;
        f.indices[2] = 2;
        mesh.faces.push_back(f);
    }

    {
        Face f;
        f.indices[0] = 2;
        f.indices[1] = 1;
        f.indices[2] = 3;
        mesh.faces.push_back(f);
    }
}

  void generate_mesh_from_cube(Mesh& mesh, const ntk::Rect3f& cube)
  {

      const int links[12][3] = { {0, 1, 3},
                                 {0, 3, 2},

                                 {0, 5, 1},
                                 {0, 4, 5},

                                 {3, 1, 5},
                                 {3, 5, 7},

                                 {2, 3, 7},
                                 {2, 7, 6},

                                 {6, 5, 4},
                                 {6, 7, 5},

                                 {0, 2, 6},
                                 {0, 6, 4} };

      Point3f center(cube.x + cube.width/2.,cube.y + cube.height/2., cube.z + cube.depth/2.);

      Point3f cube_points[8];
      Point3f proj_cube_points[8];

      const double xvals [] = {center.x-(cube.width/2), center.x+(cube.width/2)};
      const double yvals [] = {center.y-(cube.height/2), center.y+(cube.height/2)};
      const double zvals [] = {center.z-(cube.depth/2), center.z+(cube.depth/2)};

      int first_vertex_index = mesh.vertices.size();
      for (int i = 0; i < 2; ++i)
      for (int j = 0; j < 2; ++j)
      for (int k = 0; k < 2; ++k)
      {
        Point3f p(xvals[i], yvals[j], zvals[k]);
        mesh.vertices.push_back(p);
      }

      for (int f = 0; f < 12; ++f)
      {
        Face face;
        for (int i = 0; i < 3; ++i)
        {
          face.indices[i] = first_vertex_index + links[f][i];

        }
        mesh.faces.push_back(face);
      }
    }




void Mesh::addCube(const cv::Point3f& center, const cv::Point3f& sizes, const cv::Vec3b& color)
  {
    const int links[12][3] = { {0, 1, 3},
                               {0, 3, 2},

                               {0, 5, 1},
                               {0, 4, 5},

                               {3, 1, 5},
                               {3, 5, 7},

                               {2, 3, 7},
                               {2, 7, 6},

                               {6, 5, 4},
                               {6, 7, 5},

                               {0, 2, 6},
                               {0, 6, 4} };

    const bool has_colors = hasColors();

    Point3f cube_points[8];
    Point3f proj_cube_points[8];

    const double xvals [] = {center.x-(sizes.x/2), center.x+(sizes.x/2)};
    const double yvals [] = {center.y-(sizes.y/2), center.y+(sizes.y/2)};
    const double zvals [] = {center.z-(sizes.z/2), center.z+(sizes.z/2)};

    int first_vertex_index = vertices.size();
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 2; ++k)
            {
                Point3f p(xvals[i], yvals[j], zvals[k]);
                vertices.push_back(p);
                if (has_colors)
                    colors.push_back(color);
            }

    for (int f = 0; f < 12; ++f)
    {
        Face face;
        for (int i = 0; i < 3; ++i)
        {
            face.indices[i] = first_vertex_index + links[f][i];
        }
        faces.push_back(face);
    }
}

void Mesh :: addMesh(const ntk::Mesh& rhs)
{
    if (vertices.size() < 1)
    {
        *this = rhs;
        return;
    }

    // must check it before modifying it.
    bool has_colors = hasColors();

    int offset = vertices.size();
    vertices.insert(vertices.end(), rhs.vertices.begin(), rhs.vertices.end());
    if (has_colors)
    {
        ntk_throw_exception_if(!rhs.hasColors(), "Cannot merge different kind of meshes.");
        colors.insert(colors.end(), rhs.colors.begin(), rhs.colors.end());
    }

    bool has_normals = hasNormals();
    if (has_normals)
    {
        ntk_throw_exception_if(!rhs.hasNormals(), "Cannot merge different kind of meshes.");
        normals.insert(normals.end(), rhs.normals.begin(), rhs.normals.end());
    }

    for (int i = 0; i < rhs.faces.size(); ++i)
    {
        Face face = rhs.faces[i];
        face.indices[0] += offset;
        face.indices[1] += offset;
        face.indices[2] += offset;
        faces.push_back(face);
    }
}

void Mesh::
applyScaleTransform(float x_scale, float y_scale, float z_scale)
{
    foreach_idx(i, vertices)
    {
        vertices[i].x *= x_scale;
        vertices[i].y *= y_scale;
        vertices[i].z *= z_scale;
    }
}

void Mesh::computeNormalsFromFaces()
{
    if (!hasFaces())
        return;

    normals.clear();
    normals.resize(vertices.size(), Vec3f(0,0,0));
    foreach_idx(i, faces)
    {
        const Face& face = faces[i];
        Vec3f v01 = vertices[face.indices[1]] - vertices[face.indices[0]];
        Vec3f v02 = vertices[face.indices[2]] - vertices[face.indices[0]];
        Vec3f n = v01.cross(v02);
        for (int k = 0; k < face.numVertices(); ++k)
            normals[face.indices[k]] += Point3f(n);
    }

    foreach_idx(i, normals)
    {
        Vec3f v = normals[i];
        ntk::normalize(v);
        normals[i] = v;
    }
}

void Mesh::invertFaceNormals()
{
    foreach_idx(face_i, faces)
    {
        std::swap (faces[face_i].indices[0], faces[face_i].indices[2]);
    }
}

void Mesh::duplicateSharedVertices()
{
    // FIXME: implement
    abort ();
}

void Mesh::computeVertexFaceMap(std::vector< std::vector<int> >& faces_per_vertex) const
{
    faces_per_vertex.resize(vertices.size());
    foreach_idx(face_i, faces)
    {
        for (int v_i = 0; v_i < 3; ++v_i)
        {
            faces_per_vertex[faces[face_i].indices[v_i]].push_back(face_i);
        }
    }
}

void Mesh::computeFaceNeighbors(std::vector< std::vector<int> >& faces_neighbors,
                                const std::vector< std::vector<int> >& faces_per_vertex) const
{
    faces_neighbors.resize(faces.size());

    foreach_idx(face_i, faces)
    {
        for (int v_i = 0; v_i < 3; ++v_i)
        {
            int vertex_index = faces[face_i].indices[v_i];
            const std::vector<int>& faces_for_this_vertex = faces_per_vertex[vertex_index];
            foreach_idx(neighb_i, faces_for_this_vertex)
            {
                const int neighb_face = faces_for_this_vertex[neighb_i];

                std::vector<int>& this_face_neighbors = faces_neighbors[face_i];

                if (neighb_face == face_i)
                    continue;

                if (std::find(stl_bounds(this_face_neighbors), neighb_face) != this_face_neighbors.end())
                    continue;

                this_face_neighbors.push_back(neighb_face);
            }
        }
    }
}

void Mesh::computeEdges(std::vector< Edge >& edges,
                        std::vector< std::vector<int> >& edges_per_vertex,
                        const std::vector< std::vector<int> >& faces_per_vertex) const
{
    edges.reserve(faces.size()*2);
    edges_per_vertex.resize(vertices.size());

    foreach_idx(face_i, faces)
    {
        const Face& face = faces[face_i];
        for (int i = 0; i < 2; ++i)
        for (int j = i+1; j < 3; ++j)
        {
            Edge e;
            e.v1 = face.indices[i];
            e.v2 = face.indices[j];
            if (e.v1 > e.v2)
                std::swap(e.v1, e.v2);

            e.f1 = face_i;
            e.f2 = -1;

            e.length = cv::norm(vertices[e.v1] - vertices[e.v2]);

            const std::vector<int>& faces_v1 = faces_per_vertex[e.v1];
            const std::vector<int>& faces_v2 = faces_per_vertex[e.v2];

            int n_edges = 0;
            bool border_edge = true;

            // Look for the other common face.
            for (int k = 0; k < faces_v1.size(); ++k)
            {
                // We want the other face.
                if (faces_v1[k] == face_i)
                    continue;

                for (int l = 0; l < faces_v2.size(); ++l)
                {
                    if (faces_v1[k] != faces_v2[l])
                        continue;

                    // make sure we do not consider it as a border edge later.
                    border_edge = false;

                    if (faces_v1[k] < e.f1)
                        continue; // already added when processing f1.

                    e.f2 = faces_v1[k];
                    int edge_index = edges.size();
                    edges_per_vertex[e.v1].push_back(edge_index);
                    edges_per_vertex[e.v2].push_back(edge_index);
                    edges.push_back(e);
                    ++n_edges;
                }
            }

            if (n_edges > 1)
                ntk_dbg(1) << "Warning: not two-manifold.";

            if (!border_edge)
                continue; // already added

            // No second adjacent face, add the border edge.
            int edge_index = edges.size();
            edges_per_vertex[e.v1].push_back(edge_index);
            edges_per_vertex[e.v2].push_back(edge_index);
            edges.push_back(e);
        }
    }
}

void Mesh::extractConnectedComponents(std::vector<Patch>& patches,
                                      const std::vector< std::vector<int> >& faces_neighbors) const
{
    if (!hasFaceLabels())
    {
        ntk_dbg(1) << "No face labels to extract connected components.";
        return;
    }

    std::set<int> processed_inner_faces;
    foreach_idx(face_i, face_labels)
    {
        if (processed_inner_faces.find(face_i) != processed_inner_faces.end())
            continue;

        int patch_label = face_labels[face_i];

        Patch patch;
        patch.label = patch_label;

        std::set<int> processed_border_faces;
        std::queue<int> to_explore;
        to_explore.push(face_i);
        processed_inner_faces.insert(face_i);
        // Looking for a new patch.
        while (!to_explore.empty())
        {
            int patch_face_i = to_explore.front();
            to_explore.pop();

            int label = face_labels[patch_face_i];

            if (label == patch_label)
            {
                patch.inner_faces.push_back(patch_face_i);

                foreach_idx(neighb_i, faces_neighbors[patch_face_i])
                {
                    int neighb = faces_neighbors[patch_face_i][neighb_i];

                    if (face_labels[neighb] != patch_label)
                    {
                        if (std::find(stl_bounds(patch.border_faces), patch_face_i) == patch.border_faces.end())
                            patch.border_faces.push_back(patch_face_i);

                        if (std::find(stl_bounds(patch.outer_faces), neighb) == patch.outer_faces.end())
                            patch.outer_faces.push_back(neighb);
                    }
                    else
                    {
                        if (processed_inner_faces.find(neighb) != processed_inner_faces.end())
                            continue;
                        to_explore.push(neighb);
                        processed_inner_faces.insert(neighb);
                    }
                }
            }
        }

        if (patch.inner_faces.size() > 0)
            patches.push_back(patch);
    } // patch

    ntk_dbg_print(patches.size(), 1);

    // Compute inner and frontier vertices.
    foreach_idx(patch_i, patches)
    {
        Patch& patch = patches[patch_i];

        foreach_idx(inner_face_i, patch.inner_faces)
        {
            int face_i = patch.inner_faces[inner_face_i];
            const Face& face = faces[face_i];

            foreach_idx(v_i, face)
            {
                int vertex = face.indices[v_i];
                patch.inner_vertices.insert(vertex);
            }
        }

        foreach_idx(outer_face_i, patch.outer_faces)
        {
            int face_i = patch.outer_faces[outer_face_i];
            const Face& face = faces[face_i];

            foreach_idx(v_i, face)
            {
                int vertex = face.indices[v_i];
                patch.outer_vertices.insert(vertex);
            }
        }

        std::set_intersection(stl_bounds(patch.inner_vertices),
                              stl_bounds(patch.outer_vertices),
                              std::inserter(patch.border_vertices, patch.border_vertices.begin()));
    } // patch
}

float Mesh::computeLength(const Edge &edge) const
{
    return cv::norm(vertices[edge.v1] - vertices[edge.v2]);
}

struct VertexComparator
{
    VertexComparator(const std::vector<cv::Point3f>& vertices) : vertices(vertices) {}

    bool operator()(int i1, int i2) const
    {
        return vertices[i1] < vertices[i2];
    }

    const std::vector<cv::Point3f>& vertices;
};

void Mesh::removeDuplicatedVertices(float max_dist)
{
    std::vector<int> ordered_indices (vertices.size());
    foreach_idx(i, ordered_indices) ordered_indices[i] = i;

    VertexComparator comparator (vertices);
    std::sort(stl_bounds(ordered_indices), comparator);

    std::map<int, int> vertex_alias;

    int i = 0;
    int j = i;
    for (; i < ordered_indices.size() - 1; )
    {
        j = i + 1;
        while (j < ordered_indices.size()
               && flt_eq(vertices[ordered_indices[i]], vertices[ordered_indices[j]], max_dist))
        {
            vertex_alias[ordered_indices[j]] = ordered_indices[i];
            vertices[ordered_indices[j]] = infinite_point();
            ++j;
        }
        i = j;
    }

    foreach_idx(face_i, faces)
    {
        foreach_idx(v_i, faces[face_i])
        {
            std::map<int, int>::const_iterator it = vertex_alias.find(faces[face_i].indices[v_i]);
            if (it != vertex_alias.end())
            {
                faces[face_i].indices[v_i] = it->second;
            }
        }
    }
}

void Mesh::removeIsolatedVertices()
{
    std::vector<int> new_indices (vertices.size());
    int cur_index = 0;
    foreach_idx(i, vertices)
    {
        if (isnan(vertices[i]))
        {
            new_indices[i] = -1;
        }
        else
        {
            new_indices[i] = cur_index;
            ++cur_index;
        }
    }

    ntk::Mesh new_mesh;
    new_mesh.vertices.resize(cur_index);

    if (hasColors())
        new_mesh.colors.resize(cur_index);
    if (hasNormals())
        new_mesh.normals.resize(cur_index);
    if (hasTexcoords())
        new_mesh.texcoords.resize(cur_index);

    foreach_idx(i, vertices)
    {
        if (new_indices[i] < 0) continue;
        new_mesh.vertices[new_indices[i]] = vertices[i];
        if (hasColors())
            new_mesh.colors[new_indices[i]] = colors[i];
        if (hasNormals())
            new_mesh.normals[new_indices[i]] = normals[i];
        if (hasTexcoords())
            new_mesh.texcoords[new_indices[i]] = texcoords[i];
    }

    foreach_idx(face_i, faces)
    {
        foreach_idx(v_i, faces[face_i])
        {
            int old_index = faces[face_i].indices[v_i];
            ntk_assert(new_indices[old_index] >= 0, "Inconsistent face.");
            faces[face_i].indices[v_i] = new_indices[old_index];
        }
    }

    vertices = new_mesh.vertices;
    colors = new_mesh.colors;
    normals = new_mesh.normals;
    texcoords = new_mesh.texcoords;

    if (hasFaces() && hasNormals())
        computeNormalsFromFaces();
}

struct FaceComparatorByIndices
{
    FaceComparatorByIndices(const std::vector<Face>& faces) : faces(faces) {}

    bool operator()(int i1, int i2) const
    {
        for (int k = 0; k < 3; ++k)
        {
            if (faces[i1].indices[k] != faces[i2].indices[k])
                return faces[i1].indices[k] < faces[i2].indices[k];
        }
        return false;
    }

    const std::vector<Face>& faces;
};

void Mesh::removeDuplicatedFaces()
{
    std::vector<int> ordered_indices (faces.size());
    foreach_idx(i, ordered_indices) ordered_indices[i] = i;

    foreach_idx(i, faces) faces[i].sort();

    FaceComparatorByIndices comparator (faces);
    std::sort(stl_bounds(ordered_indices), comparator);

    std::vector<Face> new_faces;
    new_faces.reserve(faces.size());

    std::vector<FaceTexcoord> new_face_texcoords;
    new_face_texcoords.reserve(face_texcoords.size());

    Face prev_face (-1, -1, -1);
    for (int i = 0; i < ordered_indices.size(); ++i)
    {
        const int face_i = ordered_indices[i];
        const Face& face = faces[face_i];
        if (face == prev_face)
        {
            ntk_dbg(2) << "Removing duplicate face.";
            continue;
        }

        prev_face = face;
        new_faces.push_back(face);
        if (hasFaceTexcoords())
            new_face_texcoords.push_back(face_texcoords[face_i]);
    }

    faces = new_faces;
    face_texcoords = new_face_texcoords;
}

void Mesh::removeNanVertices()
{
    std::vector<int> new_indices (vertices.size());
    int cur_index = 0;
    foreach_idx(i, vertices)
    {
        if (isnan(vertices[i]))
        {
            new_indices[i] = -1;
        }
        else
        {
            new_indices[i] = cur_index;
            ++cur_index;
        }
    }

    ntk::Mesh new_mesh;
    new_mesh.vertices.resize(cur_index);

    if (hasColors())
        new_mesh.colors.resize(cur_index);
    if (hasNormals())
        new_mesh.normals.resize(cur_index);
    if (hasTexcoords())
        new_mesh.texcoords.resize(cur_index);

    foreach_idx(i, vertices)
    {
        if (new_indices[i] < 0) continue;
        new_mesh.vertices[new_indices[i]] = vertices[i];
        if (hasColors())
            new_mesh.colors[new_indices[i]] = colors[i];
        if (hasNormals())
            new_mesh.normals[new_indices[i]] = normals[i];
        if (hasTexcoords())
            new_mesh.texcoords[new_indices[i]] = texcoords[i];
    }

    foreach_idx(face_i, faces)
    {
        foreach_idx(v_i, faces[face_i])
        {
            int old_index = faces[face_i].indices[v_i];
            int new_index = new_indices[old_index];
            if (new_index < 0)
            {
                faces[face_i].kill();
                break;
            }

            faces[face_i].indices[v_i] = new_index;
        }
    }

    vertices = new_mesh.vertices;
    colors = new_mesh.colors;
    normals = new_mesh.normals;

    removeDeadFaces();
}

struct FaceComparatorByArea
{
    FaceComparatorByArea(const Mesh& mesh) : mesh(mesh) {}

    bool operator()(int i1, int i2) const
    {
        return mesh.faceArea(i1) < mesh.faceArea(i2);
    }

    const Mesh& mesh;
};

void Mesh::removeNonManifoldFaces()
{
    std::vector< std::vector<int> > faces_per_vertex;
    computeVertexFaceMap(faces_per_vertex);

    foreach_idx(face_i, faces)
    {
        const Face& face = faces[face_i];
        if (!face.isValid())
            continue;

        bool face_is_dead = false;

        for (int i = 0; !face_is_dead && i < 2; ++i)
        for (int j = i+1; !face_is_dead && j < 3; ++j)
        {
            int v1 = face.indices[i];
            int v2 = face.indices[j];
            if (v1 > v2)
                std::swap(v1, v2);

            const std::vector<int>& faces_v1 = faces_per_vertex[v1];
            const std::vector<int>& faces_v2 = faces_per_vertex[v2];

            std::vector<int> adjacent_faces;
            adjacent_faces.push_back(face_i);

            // Look for the other common face.
            for (int k = 0; k < faces_v1.size(); ++k)
            {
                // We want other faces.
                if (faces_v1[k] == face_i)
                    continue;

                if (!faces[faces_v1[k]].isValid())
                    continue;

                for (int l = 0; l < faces_v2.size(); ++l)
                {
                    if (faces_v1[k] != faces_v2[l])
                        continue;

                    adjacent_faces.push_back(faces_v1[k]);
                }
            }

            if (adjacent_faces.size() <= 2)
                continue; // ok, two-manifold.

            std::sort(stl_bounds(adjacent_faces), FaceComparatorByArea(*this));
            for (int k = 0; k < adjacent_faces.size() - 2; ++k)
            {
                if (adjacent_faces[k] == face_i)
                    face_is_dead = true;
                faces[adjacent_faces[k]].kill();
            }
        }
    }

    removeDeadFaces();
}

void Mesh::removeFacesWithoutVisibility()
{
    if (!hasFaceLabels())
    {
        ntk_dbg(1) << "WARNING: no visibility info to remove faces.";
        return;
    }

    for (int i = 0; i < faces.size(); ++i)
    {
        if (face_labels[i] < 0)
            faces[i].kill();
    }

    removeDeadFaces();
}

float Mesh::faceArea(int face_id) const
{
    const Face& face = faces[face_id];
    return triangleArea(vertices[face.indices[0]], vertices[face.indices[1]], vertices[face.indices[2]]);
}

void Mesh::removeDeadFaces()
{
    std::vector<Face> new_faces;
    new_faces.reserve(faces.size());

    std::vector<FaceTexcoord> new_face_texcoords;
    new_face_texcoords.reserve(face_texcoords.size());

    std::vector<int> new_face_labels;
    new_face_labels.reserve(face_labels.size());

    foreach_idx(face_i, faces)
    {
        const Face& face = faces[face_i];
        if (!face.isValid()
                || face.indices[0] == face.indices[1]
                || face.indices[0] == face.indices[2]
                || face.indices[1] == face.indices[2])
            continue;
        new_faces.push_back(face);

        if (hasFaceTexcoords())
            new_face_texcoords.push_back(face_texcoords[face_i]);

        if (hasFaceLabels())
            new_face_labels.push_back(face_labels[face_i]);
    }

    faces = new_faces;
    face_texcoords = new_face_texcoords;
    face_labels = new_face_labels;
}

cv::Point3f barycentricCoordinates(const cv::Point3f ref_points[3], const cv::Point3f& p)
{
    // http://stackoverflow.com/questions/5066686/projecting-to-a-2d-plane-for-barycentric-calculations
    float triArea =  cv::norm((ref_points[1] - ref_points[0]).cross(ref_points[2] - ref_points[0])) * 0.5f;
    float u = (cv::norm((ref_points[1] - p).cross(ref_points[2] - p)) * 0.5f) / triArea;
    float v = (cv::norm((ref_points[0] - p ).cross(ref_points[2] - p )) * 0.5f) / triArea;
    float w = (cv::norm((ref_points[0] - p ).cross(ref_points[1] - p)) * 0.5f) / triArea;
    return cv::Point3f(u, v, w);
}

cv::Point3f fastBarycentricCoordinates(const cv::Point3f ref_points[3], const cv::Point3f& p)
{
    abort(); // does not work, e.g. with z=0 on all points.

    // http://stackoverflow.com/questions/5066686/projecting-to-a-2d-plane-for-barycentric-calculations
    float x0 = ref_points[0].x, y0 = ref_points[0].y, z0 = ref_points[0].z;
    float x1 = ref_points[1].x, y1 = ref_points[1].y, z1 = ref_points[1].z;
    float x2 = ref_points[2].x, y2 = ref_points[2].y, z2 = ref_points[2].z;
    float xp = p.x,             yp = p.y,             zp = p.z;

    float det = x0*(y1*z2 - y2*z1) + x1*(y2*z0 - z2*y0) + x2*(y0*z1 - y1*z0);
    float b0 = ((x1*y2-x2*y1)*zp + xp*(y1*z2-y2*z1) + yp*(x2*z1-x1*z2)) / det;
    float b1 = ((x2*y0-x0*y2)*zp + xp*(y2*z0-y0*z2) + yp*(x0*z2-x2*z0)) / det;
    float b2 = ((x0*y1-x1*y0)*zp + xp*(y0*z1-y1*z0) + yp*(x1*z0-x0*z1)) / det;
    return cv::Point3f(b0, b1, b2);
}

void Face::sort()
{
    const int old_indices[3] = { indices[0], indices[1], indices[2] };
    const int min_index = std::min_element(indices, indices+3) - indices;
    for (int i = 0; i < 3; ++i)
    {
        indices[i] = old_indices[(min_index+i)%3];
    }
}

} // end of ntk
