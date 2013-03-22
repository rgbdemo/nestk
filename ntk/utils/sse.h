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

#ifndef NTK_UTILS_SSE_H
#define NTK_UTILS_SSE_H

#include <vectorial/vectorial.h>
#include <opencv2/core/core.hpp>

namespace ntk
{

template <class T>
T* allocate_sse_buffer(int buf_size)
{
    T *data = 0;
    size_t num = buf_size * sizeof(T);
    num += num % 128; // make sure there is enough data.
#ifndef WIN32
    int failed = posix_memalign((void**)&data, 16, num);
    if (failed)
        return 0;
#else
    abort(); // FIXME: implement on Windows.
#endif
    return data;
}

inline cv::Point3f
toPoint3f(const vectorial::vec4f& v)
{
    return cv::Point3f(v.x(), v.y(), v.z());
}

inline vectorial::mat4f
toSSE(const cv::Mat1f& cv_mat)
{
    vectorial::mat4f m (cv_mat.ptr<float>());
    return vectorial::transpose(m);
}

inline vectorial::vec4f
toSSE(const cv::Point3f& cv_p)
{
    vectorial::vec4f p (cv_p.x, cv_p.y, cv_p.z, 1);
    return p;
}

inline vectorial::vec4f sseUnprojectFromImage(const vectorial::mat4f& m,
                                              const cv::Point3f& p)
{
    vectorial::vec4f v (p.x*p.z, p.y*p.z, p.z, 1);
    return m * v;
}

inline vectorial::vec4f sseProjectToImage(const vectorial::mat4f& m,
                                          const cv::Point3f& p)
{
    vectorial::vec4f v (p.x, p.y, p.z, 1);
    v = m * v;
    float d = 1.f/v.z();
    vectorial::vec4f proj (d, d, 1, 1);
    return v * proj;
}

inline vectorial::vec4f sseProjectToImage(const vectorial::mat4f& m,
                                          const vectorial::vec4f& p)
{
    vectorial::vec4f v = m * p;
    float d = 1.f/v.z();
    vectorial::vec4f proj (d, d, 1, 1);
    return v * proj;
}

class Pose3D;
class VectorialProjector
{
public:
    VectorialProjector(const Pose3D& pose);

public:
    inline vectorial::vec4f unprojectFromImage(const cv::Point3f& p) { return sseUnprojectFromImage(sse_unproj, p); }
    inline vectorial::vec4f projectToImage(const cv::Point3f& p) { return sseProjectToImage(sse_proj, p); }
    inline vectorial::vec4f projectToImage(const vectorial::vec4f& p) { return sseProjectToImage(sse_proj, p); }

private:
    vectorial::mat4f sse_unproj;
    vectorial::mat4f sse_proj;
};

}

#endif // NTK_UTILS_SSE_H
