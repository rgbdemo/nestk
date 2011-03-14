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

#ifdef NESTK_USE_GLEW
# include <GL/glew.h>
#endif

#include "mesh_renderer.h"

#include <ntk/numeric/utils.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/utils/time.h>

using namespace cv;

namespace ntk
{

  MeshRenderer :: MeshRenderer(int image_width, int image_height, float transparency)
    : m_mesh(0), m_transparency(transparency)
  {
    m_depth_buffer = cv::Mat1f(Size(image_width, image_height));
    m_color_buffer = cv::Mat4b(Size(image_width, image_height));
    m_context = new QGLContext(QGLFormat(QGL::SampleBuffers|QGL::DepthBuffer|QGL::AlphaChannel));
    m_pbuffer = new QGLPixelBuffer(QSize(image_width, image_height), m_context->format());
    m_pbuffer->makeCurrent();

    GLenum err = glewInit();
    ntk_ensure(err == GLEW_OK, "Could not load GLEW.");
    std::cout << "Status: Using GLEW " << glewGetString(GLEW_VERSION) << std::endl;
    ntk_assert(GLEW_ARB_vertex_buffer_object, "Cannot use vertex buffer.");

    glEnable(GL_DEPTH);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    glClearColor(1.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  }

  MeshRenderer :: ~MeshRenderer()
  {
    delete m_context;
    delete m_pbuffer;
  }

  void MeshRenderer :: setPose(const Pose3D& pose, float arg_near_plane, float arg_far_plane)
  {
    VertexBufferObject& vbo = m_vertex_buffer_object;
    pose.cvCameraTransform().copyTo(vbo.model_view_matrix);
    // Transpose the matrix for OpenGL column-major.
    vbo.model_view_matrix = vbo.model_view_matrix.t();

    float near_plane = arg_near_plane, far_plane = arg_far_plane;
    // FIXME: if (near_plane < 0 || far_plane < 0)
      estimateOptimalPlanes(pose, &near_plane, &far_plane);
    m_last_near_plane = near_plane;
    m_last_far_plane = far_plane;

    m_pbuffer->makeCurrent();
    glMatrixMode (GL_MODELVIEW);
    glLoadIdentity ();
    cv::Vec3f euler_angles = pose.cvEulerRotation();
    glTranslatef(pose.cvTranslation()[0], pose.cvTranslation()[1], pose.cvTranslation()[2]);
    glRotatef(euler_angles[2]*180.0/M_PI, 0, 0, 1);
    glRotatef(euler_angles[1]*180.0/M_PI, 0, 1, 0);
    glRotatef(euler_angles[0]*180.0/M_PI, 1, 0, 0);

    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    double dx = pose.imageCenterX() - (m_pbuffer->width() / 2.0);
    double dy = pose.imageCenterY() - (m_pbuffer->height() / 2.0);
    glViewport(dx, -dy, m_pbuffer->width(), m_pbuffer->height());
    if (pose.isOrthographic())
    {
      gluOrtho2D(-pose.focalX()/2, pose.focalX()/2, -pose.focalY()/2, pose.focalY()/2);
    }
    else
    {
      double fov = (180.0/M_PI) * 2.0*atan(m_pbuffer->height()/(2.0*pose.focalY()));
      // double fov2 = (180.0/M_PI) * 2.0*atan(image.cols/(2.0*pose.focalX()));
      // ntk_dbg_print(fov2, 2);
      // gluPerspective(fov2,  double(image.rows)/image.cols, near_plane, far_plane);
      gluPerspective(fov, double(m_pbuffer->width())/m_pbuffer->height(), near_plane, far_plane);
    }

    glMatrixMode (GL_MODELVIEW);
  }

  void MeshRenderer :: clearVertexBufferObject()
  {
    if (!m_vertex_buffer_object.initialized)
      return;

    GLuint vboId = m_vertex_buffer_object.vertex_id;
    glDeleteBuffersARB(1, &vboId);
    if (m_vertex_buffer_object.has_faces)
    {
      vboId = m_vertex_buffer_object.faces_id;
      glDeleteBuffersARB(1, &vboId);
    }

    if (m_vertex_buffer_object.has_texcoords)
    {
      glDeleteTextures(1, &m_vertex_buffer_object.texture_id);
    }
  }

  void MeshRenderer :: setMesh(const Mesh& mesh)
  {
    m_pbuffer->makeCurrent();

    clearVertexBufferObject();
    m_mesh = &mesh;

    GLuint vbo_id = -1, vbo_faces_id = -1;
    glGenBuffersARB(1, &vbo_id);
    if (mesh.hasFaces())
      glGenBuffersARB(1, &vbo_faces_id);

    VertexBufferObject& vbo = m_vertex_buffer_object;
    vbo.nb_faces = 0;
    vbo.vertex_id = vbo_id;
    vbo.faces_id = vbo_faces_id;
    vbo.has_faces = mesh.hasFaces();
    vbo.has_color = mesh.hasColors();
    vbo.has_texcoords = mesh.hasTexcoords();
    vbo.color_offset = mesh.vertices.size()*sizeof(Vec3f);
    vbo.texture_offset = mesh.vertices.size()*sizeof(Vec3f) + mesh.colors.size() * sizeof(Vec3b);
    vbo.nb_vertices = mesh.vertices.size();

    glBindBufferARB(GL_ARRAY_BUFFER_ARB, vbo.vertex_id);
    glBufferDataARB(GL_ARRAY_BUFFER_ARB,
                    mesh.colors.size()*sizeof(Vec3b)
                    + mesh.vertices.size()*sizeof(Vec3f)
                    + mesh.texcoords.size()*sizeof(Point2f), // size
                    0, // null pointer: just allocate memory
                    GL_STATIC_DRAW_ARB);
    glBufferSubDataARB(GL_ARRAY_BUFFER_ARB, 0, mesh.vertices.size()*sizeof(Vec3f), &mesh.vertices[0]);

    if (vbo.has_texcoords)
      glBufferSubDataARB(GL_ARRAY_BUFFER_ARB, vbo.texture_offset, mesh.texcoords.size()*sizeof(Point2f), &mesh.texcoords[0]);
    else if (vbo.has_color)
      glBufferSubDataARB(GL_ARRAY_BUFFER_ARB, vbo.color_offset, mesh.colors.size()*sizeof(Vec3b), &mesh.colors[0]);

    if (vbo.has_faces)
    {
      vbo.nb_faces = mesh.faces.size();
      glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, vbo.faces_id);
      glBufferDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB,
                      mesh.faces.size() * 3 * sizeof(GLuint), // size
                      (GLuint*)&mesh.faces[0],
                      GL_STATIC_DRAW_ARB);
    }

    if (mesh.texture.data)
    {
      glGenTextures( 1, &vbo.texture_id );
      glBindTexture( GL_TEXTURE_2D, vbo.texture_id );
      glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE );
      glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
      //glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_BASE_LEVEL,0);
      //glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAX_LEVEL,0);
      //glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
      //glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
      glTexImage2D(GL_TEXTURE_2D, 0,
                   GL_RGB8, mesh.texture.cols, mesh.texture.rows,
                   0, GL_BGR, GL_UNSIGNED_BYTE,mesh.texture.data);
    }   

    vbo.initialized = true;
  }

  void MeshRenderer :: estimateOptimalPlanes(const Pose3D& pose, float* near_plane, float* far_plane)
  {
    float min_z = std::numeric_limits<float>::max();
    float max_z = 0.01;

    for (int i = 0; i < m_mesh->faces.size(); ++i)
    {
      const Point3f& v1 = m_mesh->vertices[m_mesh->faces[i].indices[0]];
      const Point3f& v2 = m_mesh->vertices[m_mesh->faces[i].indices[1]];
      const Point3f& v3 = m_mesh->vertices[m_mesh->faces[i].indices[2]];

      Point3f pv1 = pose.cameraTransform(v1);
      Point3f pv2 = pose.cameraTransform(v2);
      Point3f pv3 = pose.cameraTransform(v3);

      min_z = ntk::math::min(min_z,-pv1.z);
      min_z = ntk::math::min(min_z,-pv2.z);
      min_z = ntk::math::min(min_z,-pv3.z);

      max_z = ntk::math::max(max_z,-pv1.z);
      max_z = ntk::math::max(max_z,-pv2.z);
      max_z = ntk::math::max(max_z,-pv3.z);
    }

    ntk_dbg_print(min_z, 2);
    ntk_dbg_print(max_z, 2);

    if (min_z < 0)
      min_z = 0.01;

    if (max_z < min_z)
      max_z = (min_z*2);

    *near_plane = min_z*0.9;
    *far_plane = max_z*1.1;
  }

  void MeshRenderer :: computeDepthBuffer()
  {
    glReadPixels(0, 0, m_depth_buffer.cols, m_depth_buffer.rows, GL_DEPTH_COMPONENT, GL_FLOAT, m_depth_buffer.data);
    cv::Mat1f flipped;
    flip(m_depth_buffer, flipped, 0);
    m_depth_buffer = flipped;

    // FIXME: this is very slow !!!
    // gluUnproject is not necessary, or at least one
    // could invert the projection Matrix only once.

    cv::Mat_<GLdouble> modelMatrix(4,4);
    setIdentity(modelMatrix);

    cv::Mat_<GLdouble> projMatrix(4,4);
    glGetDoublev(GL_PROJECTION_MATRIX,projMatrix[0]);
    // projMatrix = projMatrix.inv();

    int viewport[4];
    glGetIntegerv(GL_VIEWPORT,viewport);

    GLdouble objx, objy, objz;

    for (int r = 0; r < m_depth_buffer.rows; ++r)
      for (int c = 0; c < m_depth_buffer.cols; ++c)
      {
      double depth = m_depth_buffer(r,c);
      if (ntk::flt_eq(depth,1) || ntk::flt_eq(depth,0))
      {
        m_depth_buffer(r,c) = 0;
        continue;
      }
      gluUnProject(c, r, depth, modelMatrix[0], projMatrix[0], viewport,&objx, &objy, &objz);
      // double winz = (2.0*depth)-1;
      // double objz = (projMatrix(2,3)) / (winz * projMatrix(3,2) + projMatrix(3,3));
      // double objz = ;
      m_depth_buffer(r,c) = -objz;
    }
  }

  void MeshRenderer :: renderToImage(cv::Mat4b& image, int flags)
  {
    ntk_assert(m_vertex_buffer_object.initialized,
               "Renderer not initialized! Call setPose and setMesh.");

    ntk::TimeCount tc_gl_current("make_current", 2);
    m_pbuffer->makeCurrent();
    tc_gl_current.stop();

    ntk::TimeCount tc_gl_render("gl_render", 2);

    if (flags & WIREFRAME)
      glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );

    VertexBufferObject& vbo = m_vertex_buffer_object;
    glBindBufferARB(GL_ARRAY_BUFFER_ARB, vbo.vertex_id);

    if (vbo.has_texcoords)
    {
      glEnable(GL_TEXTURE_2D);
      glBindTexture(GL_TEXTURE_2D, vbo.texture_id);
    }
    else
    {
      glDisable(GL_TEXTURE_2D);
    }

    if (vbo.has_texcoords)
      glEnableClientState(GL_TEXTURE_COORD_ARRAY);

    if (vbo.has_color)
      glEnableClientState(GL_COLOR_ARRAY);
    else
      glColor3f(1.0f,0.f,0.f);

    glEnableClientState(GL_VERTEX_ARRAY);

    glVertexPointer(3, GL_FLOAT, 0, 0);
    if (vbo.has_color)
      glColorPointer(3, GL_UNSIGNED_BYTE, 0, ((char*) NULL) + vbo.color_offset);

    if (vbo.has_texcoords)
      glTexCoordPointer(2, GL_FLOAT, 0, ((char*) NULL) + vbo.texture_offset);

    if (vbo.has_faces)
    {
      glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, vbo.faces_id);
      glNormal3f(0, 0, 1);
      glDrawElements(GL_TRIANGLES, vbo.nb_faces*3, GL_UNSIGNED_INT, 0);
    }
    else
    {
      glDrawArrays(GL_POINTS,
                   0,
                   vbo.nb_vertices);
    }

    glDisableClientState(GL_VERTEX_ARRAY);

    if (vbo.has_color)
      glDisableClientState(GL_COLOR_ARRAY);

    if (vbo.has_texcoords)
      glDisableClientState(GL_TEXTURE_COORD_ARRAY);

    // bind with 0, so, switch back to normal pointer operation
    glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);

    if (vbo.has_faces)
    {
      glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, 0);
    }

    glFinish();
    tc_gl_render.stop();

    ntk::TimeCount tc_image("to_image", 2);
    QImage qimage = m_pbuffer->toImage();
    tc_image.stop();

    ntk::TimeCount tc_depth_buffer("compute_depth_buffer", 2);
    computeDepthBuffer();
    tc_depth_buffer.stop();

    ntk::TimeCount tc_convert("convert_to_cv", 2);
    for (int r = 0; r < qimage.height(); ++r)
      for (int c = 0; c < qimage.width(); ++c)
      {
      QRgb pixel = qimage.pixel(c,r);
      Vec4b color (qBlue(pixel), qGreen(pixel), qRed(pixel), qAlpha(pixel));
      m_color_buffer(r,c) = color;
      float a = qAlpha(pixel)/255.f;
      if (a > 0)
      {
        Vec4b old_color = image(r,c);
        image(r,c) = Vec4b(old_color[0]*(1-a) + color[0]*a,
                           old_color[1]*(1-a) + color[1]*a,
                           old_color[2]*(1-a) + color[2]*a,
                           255);
      }
    }    
    tc_convert.stop();
  }

} // ntk
