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

#ifndef NTK_OPENCV_UTILS_H
#define NTK_OPENCV_UTILS_H

#include <ntk/core.h>
#include <ntk/utils/debug.h>
#include <ntk/utils/serializable.h>
#include <ntk/numeric/utils.h>
#include <QString>

class QImage;

# define for_all_rc(im) \
    for (int r = 0; r < (im).rows; ++r) \
    for (int c = 0; c < (im).cols; ++c)

// depth row col iterations
# define for_all_drc(im) \
    for (int d = 0; d < (im).size[0]; ++d) \
    for (int r = 0; r < (im).size[1]; ++r) \
    for (int c = 0; c < (im).size[2]; ++c)

inline bool operator<(const cv::Point2i& p1, const cv::Point2i& p2)
{
    if (p1.x == p2.x) return p1.y < p2.y;
    return p1.x < p2.x;
}

inline bool operator<(const cv::Point3f& p1, const cv::Point3f& p2)
{
    if (p1.x != p2.x) return p1.x < p2.x;
    if (p1.y != p2.y) return p1.y < p2.y;
    return p1.z < p2.z;
}

const NtkDebug& operator<<(const NtkDebug& stream, const cv::Mat1f& m);
const NtkDebug& operator<<(const NtkDebug& stream, const cv::Mat1d& m);

namespace ntk
{

inline bool flt_eq(const cv::Point3f& p1, const cv::Point3f& p2, float max_dist)
{
    return flt_eq(p1.x, p2.x, max_dist)
            && flt_eq(p1.y, p2.y, max_dist)
            && flt_eq(p1.z, p2.z, max_dist);
}

cv::Point3f computeCentroid(const std::vector<cv::Point3f>& points);

template <class T>
class Rect3_
{
public:
    Rect3_() :
        x(0), y(0), z(0),
        width(-1), height(-1), depth(-1)
    {}

    Rect3_(T x, T y, T z, T width, T height, T depth) :
        x(x), y(y), z(z),
        width(width), height(height), depth(depth)
    {}

    void extendToInclude(const cv::Point3f& p)
    {
        if (width < T(0) || height < T(0) || depth < T(0))
        {
            x = p.x;
            y = p.y;
            z = p.z;
            width = 1e-5f;
            height = 1e-5f;
            depth = 1e-5f;
        }
        else
        {
            T min_x = std::min(x, p.x);
            T max_x = std::max(x+width, p.x);
            T min_y = std::min(y, p.y);
            T max_y = std::max(y+height, p.y);
            T min_z = std::min(z, p.z);
            T max_z = std::max(z+depth, p.z);

            x = min_x; width = max_x - min_x;
            y = min_y; height = max_y - min_y;
            z = min_z; depth = max_z - min_z;
        }
    }

    bool isEmpty() const
    {
        return width < 0 || height < 0 || depth < 0;
    }

    void moveCenterTo (const cv::Point3f& p)
    {
        cv::Point3f delta = p - centroid ();
        x += delta.x;
        y += delta.y;
        z += delta.z;
    }

    cv::Point3_<T> centroid() const
    {
        return cv::Point3_<T>(x+(width/T(2)), y+(height/T(2)), z + (depth / T(2)));
    }

    cv::Point3_<T> sizes() const
    {
        return cv::Point3_<T>(width, height, depth);
    }

    bool isPointInside(const cv::Point3f& p) const
    {
        return (p.x >= x) && (p.y >= y) && (p.z >= z)
                && (p.x <= (x+width)) && (p.y <= (y+height)) && (p.z <= (z+depth));
    }

public:
    T x,y,z,width,height,depth;
};

typedef Rect3_<float> Rect3f;
typedef cv::Rect_<float> Rect2f;

void extendToInclude(cv::Rect& rect, const cv::Point &p);
bool leftRectFitIntoRight(const cv::Rect& left, const cv::Rect& right);

template < typename StreamType, typename Type >
StreamType& operator>>(StreamType& input, Rect3_<Type>& box)
{
    Type xmin=0,ymin=0,zmin=0,xmax=0,ymax=0,zmax=0;
    input >> xmin >> ymin >> zmin >> xmax >> ymax >> zmax;
    box.x = xmin; box.width = xmax-xmin;
    box.y = ymin; box.height = ymax-ymin;
    box.z = zmin; box.depth = zmax-zmin;
    return input;
}

template < typename StreamType, typename Type >
StreamType& operator<<(StreamType& output, const Rect3_<Type>& box)
{
    output << box.x << ntk::sep() << box.y << ntk::sep() << box.z
           << ntk::sep() << (box.x + box.width)
           << ntk::sep() << (box.y + box.height)
           << ntk::sep() << (box.z + box.depth);
    return output;
}

ntk::Rect3f bounding_box(const std::vector<cv::Point3f>& points);
ntk::Rect3f readBoundingBoxFromYamlFile(const std::string& filename);
void writeBoundingBoxToYamlFile(const std::string& filename, const ntk::Rect3f& bbox);

}

namespace ntk
{

inline QString toString (const cv::Vec3f& v)
{
    return QString("(%1, %2, %3)").arg(v[0]).arg(v[1]).arg(v[2]);
}

inline cv::Point3f toPoint3f(const cv::Vec4f& v)
{ return cv::Point3f(v[0], v[1], v[2]); }

inline cv::Point2f toPoint2f(const cv::Point& p)
{ return cv::Point2f(p.x, p.y); }

inline cv::Vec3f nearest_pixel(const cv::Vec3f& p)
{ return cv::Vec3f(ntk::math::rnd(p[0]), ntk::math::rnd(p[1]), p[2]); }

bool is_yx_in_range(const cv::Mat& image, int y, int x);

template <class T>
inline T bilinear_sample(const cv::Mat_<T>& img, float y, float x)
{
    int px = (int) x; // floor of x
    int py = (int) y; // floor of y
    const T& p1 = img(y,x);
    const T& p2 = is_yx_in_range(img,   y, x+1) ? img(  y, x+1) : p1;
    const T& p3 = is_yx_in_range(img, y+1,   x) ? img(y+1,   x) : p1;
    const T& p4 = is_yx_in_range(img, y+1, x+1) ? img(y+1, x+1) : p1;

    // Calculate the weights for each pixel
    float fx = x - px;
    float fy = y - py;
    float fx1 = 1.0f - fx;
    float fy1 = 1.0f - fy;

    float w1 = fx1 * fy1;
    float w2 = fx  * fy1;
    float w3 = fx1 * fy;
    float w4 = fx  * fy;

    T r = p1*w1 + p2*w2 + p3*w3 + p4*w4;
    return r;
}

inline cv::Point2f box_center(const cv::Rect_<float>& bbox)
{
    return cv::Point2f(bbox.x + bbox.width/2.0, bbox.y + bbox.height/2.0);
}

inline cv::Vec3f infinite_point()
{
    return cv::Vec3f(std::numeric_limits<float>::quiet_NaN(),
                     std::numeric_limits<float>::quiet_NaN(),
                     std::numeric_limits<float>::quiet_NaN());
}

template <class VecT>
inline void make_infinite (VecT& v) { v[0] = std::numeric_limits<float>::quiet_NaN(); }

inline bool isnan(const cv::Point3f& p)
{ return ntk::math::isnan(p.x); }

inline bool isnan(const cv::Vec3f& p)
{ return ntk::math::isnan(p[0]); }

inline void fillWithNan(cv::Mat3f& im)
{
    std::fill((cv::Vec3f*)im.datastart, (cv::Vec3f*)im.dataend, infinite_point());
}

inline cv::Vec3f toVec3f(const cv::Vec3b& v)
{
    return cv::Vec3f(v[0], v[1], v[2]);
}

inline cv::Vec3b toVec3b(const cv::Vec3f& v)
{
    return cv::Vec3b(v[0], v[1], v[2]);
}

inline cv::Vec3b toVec3b (const cv::Matx<float, 3, 1>& v)
{
    return cv::Vec3b(v(0), v(1), v(2));
}

inline cv::Vec3b toVec3b(const cv::Vec4b& v)
{
    return cv::Vec3b(v[0], v[1], v[2]);
}

inline cv::Vec3f vec_min(const cv::Vec3f& v1, const cv::Vec3f& v2)
{
    return cv::Vec3f(std::min(v1[0], v2[0]), std::min(v1[1], v2[1]), std::min(v1[2], v2[2]));
}

inline cv::Vec3f vec_max(const cv::Vec3f& v1, const cv::Vec3f& v2)
{
    return cv::Vec3f(std::max(v1[0], v2[0]), std::max(v1[1], v2[1]), std::max(v1[2], v2[2]));
}

inline cv::Vec3f vec_round(const cv::Vec3f& v)
{
    return cv::Vec3f(ntk::math::rnd(v[0]), ntk::math::rnd(v[1]), ntk::math::rnd(v[2]));
}

template <class ScalarType>
cv::Vec3d toVec3d(const cv::Mat_<ScalarType>& m)
{
    ntk_assert(m.rows == 3 && m.cols == 1, "m is not a vector.");
    return cv::Vec3d(m(0,0), m(1,0), m(2,0));
}

template <class ScalarType>
cv::Vec2f toVec2f(const cv::Mat_<ScalarType>& m)
{
    ntk_assert((m.rows == 2 || m.rows == 3) && m.cols == 1, "m is not a vector.");
    return cv::Vec2f(m(0,0), m(1,0));
}

template <class ScalarType1, class ScalarType2>
void copyMatWithCast(cv::Mat_<ScalarType1>& dest, const cv::Mat_<ScalarType2>& src)
{
    ntk_assert(dest.size == src.size, "Size mismatch");
    for_all_rc(src)
    {
        dest(r,c) = src(r,c);
    }
}

cv::Mat1b qimage_to_opencv(const QImage& im);
void opencv_to_qimage(QImage& qim, const cv::Mat1b& im);
void opencv_to_qimage(QImage& qim, const cv::Mat3b& im);
cv::Mat4b qimage_argb_to_opencv(const QImage& im);
void opencv_to_qimage(QImage& qim, const cv::Mat4b& im);
cv::Mat4b toMat4b(const cv::Mat3b& im);
cv::Mat3b toMat3b(const cv::Mat4b& im);

void apply_mask(cv::Mat1b& im, const cv::Mat1b& mask);
void apply_mask(cv::Mat3b& im, const cv::Mat1b& mask);
void apply_mask(cv::Mat1f& im, const cv::Mat1b& mask);

void read_from_yaml(cv::FileNode node, cv::Vec3f& v);
void write_to_yaml(cv::FileStorage& output_file, const std::string& name, const cv::Vec3f& v);

void read_from_yaml(cv::FileNode node, cv::Rect& v);
void write_to_yaml(cv::FileStorage& output_file, const std::string& name, const cv::Rect& v);

void read_from_yaml(cv::FileNode node, bool& b);
void write_to_yaml(cv::FileStorage& output_file, const std::string& name, bool b);

void read_from_yaml(cv::FileNode node, int& b);
void write_to_yaml(cv::FileStorage& output_file, const std::string& name, int b);

void read_from_yaml(cv::FileNode node, double& b);
void write_to_yaml(cv::FileStorage& output_file, const std::string& name, double b);

void read_from_yaml(cv::FileNode node, cv::Mat& matrix);
void write_to_yaml(cv::FileStorage& output_file, const std::string& name, const cv::Mat& matrix);

/*!
 * Obsolete. Use write_to_yaml. This is provided for API compatility.
 */
void writeMatrix(cv::FileStorage& output_file, const std::string& name, const cv::Mat& matrix);
void readMatrix(cv::FileStorage& input_file, const std::string& name, cv::Mat& matrix);

cv::Mat1b normalize_toMat1b(const cv::Mat1f& image);
cv::Mat3b toMat3b(const cv::Mat1b& image);

void imwrite_normalized(const std::string& filename, const cv::Mat1b& m);
void imwrite_normalized(const std::string& filename, const cv::Mat1f& m);
void imshow_normalized(const std::string& window_name, const cv::Mat1f& m);

void imwrite_yml(const std::string& filename, const cv::Mat& image);
cv::Mat imread_yml(const std::string& filename);

/*! Read a float image from raw binary file. Only works for Little Endian platforms. */
cv::Mat1f imread_Mat1f_raw(const std::string& filename);
/*! Write a float image to binary file. Only works for Little Endian platforms. */
void imwrite_Mat1f_raw(const std::string& filename, const cv::Mat1f& m);

/*! Read a 16 bits image from raw binary file. Only works for Little Endian platforms. */
cv::Mat1w imread_Mat1w_raw(const std::string& filename);
/*! Write a 16 bits image to binary file. Only works for Little Endian platforms. */
void imwrite_Mat1w_raw(const std::string& filename, const cv::Mat1w& m);

/*! Read a 16 bits image from raw binary file, compressed in LZF. Only works for Little Endian platforms. */
cv::Mat1w imread_Mat1w_lzf(const std::string& filename);
/*! Write a 16 bits image to binary file, compressed in LZF. Only works for Little Endian platforms. */
void imwrite_Mat1w_lzf(const std::string& filename, const cv::Mat1w& m);

/*! Read a 16 bits image from raw binary file, transformed into gradient and compressed in LZF. Only works for Little Endian platforms. */
cv::Mat1w imread_Mat1w_grad_lzf(const std::string& filename);
/*! Write a 16 bits image to binary file, transformed into gradient and compressed in LZF. Only works for Little Endian platforms. */
void imwrite_Mat1w_grad_lzf(const std::string& filename, const cv::Mat1w& m);

/*! Read a 16 bits image from raw binary file, compressed with OpenNI + LZF. Only works for Little Endian platforms. */
cv::Mat1w imread_Mat1w_openni_lzf(const std::string& filename);
/*! Write a 16 bits image to binary file, compressed with OpenNI + LZF. Only works for Little Endian platforms. */
void imwrite_Mat1w_openni_lzf(const std::string& filename, const cv::Mat1w& m);

/*! Read a 16 bits image from raw binary file, compressed with OpenNI. Only works for Little Endian platforms. */
cv::Mat1w imread_Mat1w_openni(const std::string& filename);
/*! Write a 16 bits image to binary file, compressed with OpenNI. Only works for Little Endian platforms. */
void imwrite_Mat1w_openni(const std::string& filename, const cv::Mat1w& m);

inline cv::Mat getCvByteImage(int width, int height) { return cv::Mat(height, width, CV_8UC1); }
inline cv::Mat getCvFloatImage(int width, int height) { return cv::Mat(height, width, CV_32FC1); }
inline cv::Mat getCvColorByteImage(int width, int height) { return cv::Mat(height, width, CV_8UC3); }
inline cv::Mat getCvColorFloatImage(int width, int height) { return cv::Mat(height, width, CV_32FC3); }

double overlap_ratio(const cv::Rect_<float>& r1, const cv::Rect_<float>& r2);

inline bool is_yx_in_range(const cv::Mat& image, int y, int x)
{ return (x >= 0) && (y >= 0) && (x < image.cols) && (y < image.rows); }

inline bool is_zyx_in_range(const cv::Mat& image, int z, int y, int x)
{ return (x >= 0) && (y >= 0) && (z >= 0) && (x < image.size[0]) && (y < image.size[1]) && (z < image.size[2]); }

inline void normalize(cv::Vec3f& v) { v *= float(1.0 / (sqrt(v.dot(v)))); }
inline void normalize(cv::Vec4f& v) { v *= float(1.0 / (sqrt(v.dot(v)))); }
inline void normalize(cv::Point3f& v) { v *= float(1.0 / (sqrt(v.dot(v)))); }

inline cv::Vec3b bgr_to_rgb(const cv::Vec3b& v)
{ return cv::Vec3b(v[2], v[1], v[0]); }

void adjustRectToImage(cv::Rect& rect, const cv::Size& image_size);

inline cv::Mat3b rotate90(const cv::Mat3b& im)
{
    cv::Mat3b res = im.t();
    cv::flip(res, res, 1);
    return res;
}

float triangleArea(const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p3);
float triangleArea(const cv::Point3f& p1, const cv::Point3f& p2, const cv::Point3f& p3);

inline float normalAngles(const cv::Vec3f& n1, const cv::Vec3f& n2)
{
    return acos(n1.dot(n2));
}

inline bool normalsAreCompatible(float normal_angle, float max_degrees)
{
    return (normal_angle < (max_degrees*M_PI/180.0));
}

}

namespace cv     // FIXME: has to be put in its
// own namespace to avoid ambiguities with std::ofstream << int
{

inline QTextStream& operator<<(QTextStream& output, const cv::Point2i& point)
{
    output << point.x << ntk::sep() << point.y;
    return output;
}

inline QTextStream& operator>>(QTextStream& input, cv::Point2i& point)
{
    int x=0, y=0;
    input >> x >> y;
    point = cv::Point2i(x, y);
    return input;
}

inline QTextStream& operator<<(QTextStream& output, const cv::Point2f& point)
{
    output << point.x << ntk::sep() << point.y;
    return output;
}

inline  QTextStream& operator>>(QTextStream& input, cv::Point2f& point)
{
    float x=0, y=0;
    input >> x >> y;
    point = cv::Point2f(x, y);
    return input;
}

inline QTextStream& operator<<(QTextStream& output, const cv::Point3f& point)
{
    output << point.x << ntk::sep() << point.y << ntk::sep() << point.z;
    return output;
}

inline  QTextStream& operator>>(QTextStream& input, cv::Point3f& point)
{
    input >> point.x >> point.y >> point.z;
    return input;
}

inline QTextStream& operator<<(QTextStream& output, const cv::Vec3f& point)
{
    output << point[0] << ntk::sep() << point[1] << ntk::sep() << point[2];
    return output;
}

inline QTextStream& operator<<(QTextStream& output, const cv::Vec2f& point)
{
    output << point[0] << ntk::sep() << point[1];
    return output;
}

inline QTextStream& operator>>(QTextStream& input, cv::Vec2f& point)
{
    input >> point[0] >> point[1];
    return input;
}

inline QTextStream& operator>>(QTextStream& input, cv::Vec3f& point)
{
    double x=0,y=0,z=0;
    input >> x >> y >> z;
    point = cv::Vec3f(x,y,z);
    return input;
}

template <class T>
QTextStream& operator<<(QTextStream& output, const cv::Point_<T>& point)
{
    output << point.x << ntk::sep() << point.y;
    return output;
}

template <class T>
QTextStream& operator>>(QTextStream& input, cv::Point_<T>& point)
{
    T x=0,y=0;
    input >> x >> y;
    point = cv::Point_<T>(x,y);
    return input;
}

template <class T>
QTextStream& operator<<(QTextStream& output, const cv::Rect_<T>& r)
{
    output << r.x << ntk::sep() << r.y << ntk::sep() << r.width << ntk::sep() << r.height;
    return output;
}

template <class T>
QTextStream& operator>>(QTextStream& input, cv::Rect_<T>& r)
{
    T x=0,y=0,width=0,height=0;
    input >> x >> y >> width >> height;
    r = cv::Rect_<T>(x,y,width,height);
    return input;
}

}

namespace cv
{

inline const NtkDebug& operator<<(const NtkDebug& os, const cv::Point3f& p)
{
    os << "[" << p.x << " " << p.y << " " << p.z << "]";
    return os;
}

inline const NtkDebug& operator<<(const NtkDebug& os, const cv::Vec3f& p)
{
    os << "[" << p[0] << " " << p[1] << " " << p[2] << "]";
    return os;
}

inline const NtkDebug& operator<<(const NtkDebug& os, const cv::Vec4f& p)
{
    os << "[" << p[0] << " " << p[1] << " " << p[2] << " " << p[3] << "]";
    return os;
}

inline const NtkDebug& operator<<(const NtkDebug& os, const cv::Point2f& p)
{
    os << "[" << p.x << " " << p.y << "]";
    return os;
}

inline const NtkDebug& operator<<(const NtkDebug& os, const cv::Point2i& p)
{
    os << "[" << p.x << " " << p.y << "]";
    return os;
}

inline const NtkDebug& operator<<(const NtkDebug& os, const cv::Vec2f& p)
{
    os << "[" << p[0] << " " << p[1] << "]";
    return os;
}

template <class T>
const NtkDebug& operator<<(const NtkDebug& os, const cv::Rect_<T>& r)
{
    os << "[x=" << r.x << " y=" << r.y << " w=" << r.width << " h=" << r.height;
    return os;
}

} // namespace cv

#endif // NTK_OPENCV_UTILS_H
