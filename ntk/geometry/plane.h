#ifndef NTK_GEOMETRY_PLANE_H
#define NTK_GEOMETRY_PLANE_H

#include <ntk/core.h>

namespace ntk
{

class Plane
{
public:
  Plane(double a, double b, double c, double d)
   : a(a), b(b), c(c), d(d)
  {}

  /*!
   * Construct a plane from a normal vector and a point.
   */
  Plane(const cv::Vec3f& normal, const cv::Point3f p);

  /*!
   * Construct a plane using three points.
   */
  Plane (const cv::Point3f& p1, const cv::Point3f& p2, const cv::Point3f& p3);

  Plane() : a(0), b(0), c(0), d(0)
  {}

  bool isValid() const;

  cv::Vec3f normal() const;

  void set (double a_, double b_, double c_, double d_)
  { a = a_; b = b_; c = c_; d = d_; }

  cv::Point3f intersectionWithLine (const cv::Point3f& p1, const cv::Point3f& p2) const;

  float distanceToPlane(const cv::Point3f& p) const;

  float signedDistanceToPlane(const cv::Point3f& p) const;

  double a,b,c,d;
};

void orthogonal_basis(cv::Vec3f& v1, cv::Vec3f& v2, const cv::Vec3f& v0);

void read_from_yaml(cv::FileNode node, Plane& p);
void write_to_yaml(cv::FileStorage& output_file, const std::string& name, const Plane& p);

} // ntk

#endif // NTK_GEOMETRY_PLANE_H
