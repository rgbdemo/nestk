
#include <ntk/ntk.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/utils/sse.h>

using namespace ntk;
using namespace cv;

class NormalComputer
{
public:
    NormalComputer(const Pose3D& pose)
    {
        one_over_fx = 1.f / pose.focalX();
        one_over_fy = 1.f / pose.focalY();
        one_over_fx_mul_fy = one_over_fx * one_over_fy;
        one_over_fx_mul_fx = one_over_fx * one_over_fx;
        one_over_fy_mul_fy = one_over_fy * one_over_fy;
    }

    cv::Vec3f estimateNormal(const Mat1f& depth_yml,
                             int r, int c,
                             float depth_delta_limit)
    {
        if (!is_yx_in_range(depth_yml, r-1, c-1)
                || !is_yx_in_range(depth_yml, r+1, c+1))
            return infinite_point();

        float depth = depth_yml(r,c);
        if (depth < 1e-5)
            return infinite_point();

        float dx=0, dy=0;

        float depth_L = depth_yml(r,c-1);
        float depth_R = depth_yml(r,c+1);
        float depth_S = depth_yml(r+1,c);
        float depth_N = depth_yml(r-1,c);

        if (depth_L < 1e-5 || depth_R < 1e-5
                || depth_S < 1e-5 || depth_N < 1e-5)
            return infinite_point();

        if (std::abs(depth_L - depth) > depth_delta_limit)
            return infinite_point();

        if (std::abs(depth_R - depth) > depth_delta_limit)
            return infinite_point();

        if (std::abs(depth_N - depth) > depth_delta_limit)
            return infinite_point();

        if (std::abs(depth_S - depth) > depth_delta_limit)
            return infinite_point();

        dx = ((depth - depth_R) - (depth - depth_L)) * 0.5;
        dy = ((depth - depth_N) - (depth - depth_S)) * 0.5;

        vectorial::vec3f v1 (one_over_fx, 0, dx);
        vectorial::vec3f v2 (0, one_over_fy, dy);
        vectorial::vec3f camera_normal (-dx*one_over_fy, -one_over_fx*dy, one_over_fx_mul_fy);
        // vectorial::vec3f camera_normal = vectorial::cross(v1, v2);
        float norm = one_over_fy_mul_fy * dx * dx
                + one_over_fx_mul_fx * dy * dy
                + one_over_fx_mul_fx * one_over_fy_mul_fy;
        norm = 1.0f / sqrt(norm);
        return cv::Vec3f(camera_normal.x()*norm, camera_normal.y()*norm, camera_normal.z()*norm);
    }

    cv::Vec3f estimateHalfNormal(const Mat1f& depth_yml,
                                 int r, int c,
                                 float depth_delta_limit)
    {
        if (!is_yx_in_range(depth_yml, r-1, c-1))
            return infinite_point();

        float depth = depth_yml(r,c);
        if (depth < 1e-5)
            return infinite_point();

        float dx=0, dy=0;

        float depth_L = depth_yml(r,c-1);
        float depth_N = depth_yml(r-1,c);

        if (depth_L < 1e-5 || depth_N < 1e-5)
            return infinite_point();

        if (std::abs(depth_L - depth) > depth_delta_limit)
            return infinite_point();

        if (std::abs(depth_N - depth) > depth_delta_limit)
            return infinite_point();

        dx = depth_L - depth;
        dy = depth - depth_N;

        vectorial::vec3f v1 (one_over_fx, 0, dx);
        vectorial::vec3f v2 (0, one_over_fy, dy);
        vectorial::vec3f camera_normal (-dx*one_over_fy, -one_over_fx*dy, one_over_fx_mul_fy);
        // vectorial::vec3f camera_normal = vectorial::cross(v1, v2);
        float norm = one_over_fy_mul_fy * dx * dx
                + one_over_fx_mul_fx * dy * dy
                + one_over_fx_mul_fx * one_over_fy_mul_fy;
        norm = 1.0f / sqrt(norm);
        return cv::Vec3f(camera_normal.x()*norm, camera_normal.y()*norm, camera_normal.z()*norm);
    }

private:
    float one_over_fx, one_over_fy;
    float one_over_fx_mul_fx, one_over_fy_mul_fy, one_over_fx_mul_fy;
};

int main(int argc, char** argv)
{
    ntk::ntk_debug_level = 1;

    const int nb_it = 1;
    const int rows = 480;
    const int cols = 640;
    cv::Mat1f depth_map (rows, cols, allocate_sse_buffer<float>(rows*cols));
    for_all_rc(depth_map)
    {
        depth_map(r,c) = 0.1 * ((r+1)+(c+1));
    }

    ntk::Pose3D depth_pose;
    depth_pose.setCameraParameters(depth_map.cols, depth_map.rows, depth_map.cols/2.f, depth_map.rows/2.f);
    depth_pose.applyTransformBefore(cv::Vec3f(0.1, 0.2, 0.3), cv::Vec3f(0.2, 0.1, 0.5));
    ntk::Pose3D rgb_pose = depth_pose;
    rgb_pose.applyTransformAfter(cv::Vec3f(-0.1,0,0), cv::Vec3f(0.05,0,0));

    ntk::TimeCount tc ("normal", 1);
    cv::Point3f sum (0,0,0);
    for_all_rc(depth_map)
    {
        cv::Point3f p = (cv::Point3f) estimate_normal_from_depth(depth_map, depth_pose, r, c, FLT_MAX);
        if (!ntk::math::isnan(p.x))
            sum += p;
    }
    tc.stop();
    ntk_dbg_print(sum, 1);

    ntk::TimeCount tc_sse ("sse", 1);
    NormalComputer computer (depth_pose);
    sum = cv::Point3f(0,0,0);
    for_all_rc(depth_map)
    {
        cv::Point3f p = (cv::Point3f) computer.estimateHalfNormal(depth_map, r, c, FLT_MAX);
        if (!ntk::math::isnan(p.x))
            sum += p;
    }
    tc_sse.stop();
    ntk_dbg_print(sum, 1);
}
