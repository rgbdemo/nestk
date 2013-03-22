#ifndef NTK_GESTURE_NI_UTILS_H
#define NTK_GESTURE_NI_UTILS_H

inline cv::Point3f toPoint3f(const XnPoint3D& p)
{
  return cv::Point3f(p.X, p.Y, p.Z);
}

inline XnPoint3D toXnPoint3D(const cv::Point3f& p)
{
    XnPoint3D xp;
    xp.X = p.x;
    xp.Y = p.y;
    xp.Z = p.z;
    return xp;
}

#endif // NTK_GESTURE_NI_UTILS_H
