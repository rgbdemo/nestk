
#include <ntk/ntk.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/utils/sse.h>
#include <vectorial/vectorial.h>

#include <cstdlib>

using namespace ntk;

void test_volume()
{
    ntk::TimeCount tc ("big volumes", 1);
    const int dim = 512;
    float* buffer = allocate_sse_buffer<float>(dim*dim*dim);
    for (int d = 0; d < dim; ++d)
    for (int r = 0; r < dim; ++r)
    for (int c = 0; c < dim; ++c)
    {
        buffer[d*dim*dim+r*dim+c] = r*d*c;
    }
    tc.elapsedMsecs(" -- set_volume ");

    ntk::Pose3D depth_pose;
    depth_pose.setCameraParameters(dim, dim, dim/2.f, dim/2.f);
    depth_pose.applyTransformBefore(cv::Vec3f(0.1, 0.2, 0.3), cv::Vec3f(0.2, 0.1, 0.5));
    ntk::Pose3D rgb_pose = depth_pose;
    rgb_pose.applyTransformAfter(cv::Vec3f(-0.1,0,0), cv::Vec3f(0.05,0,0));

    VectorialProjector depth_projector (depth_pose);
    vectorial::vec4f sse_sum (0,0,0,1);
    for (int i = 0; i < 1; ++i)
    {
        for (int d = 0; d < dim; ++d)
        for (int r = 0; r < dim; ++r)
        for (int c = 0; c < dim; ++c)
        {
            if (buffer[d*dim*dim+r*dim+c] < 0)
            {
                vectorial::vec4f p3d = depth_projector.unprojectFromImage(cv::Point3f(c, r, d));
                sse_sum += p3d;
            }
        }
    }

    tc.elapsedMsecs(" -- project all points ");
    ntk_dbg_print(toPoint3f(sse_sum), 1);
}

int main()
{
    ntk::ntk_debug_level = 1;

    test_volume();
    return 0;

    const int nb_it = 1;
    const int rows = 480;
    const int cols = 640;
    cv::Mat1f depth_map (rows, cols, allocate_sse_buffer<float>(rows*cols));
    for_all_rc(depth_map)
    {
        depth_map(r,c) = (r+1)*(c+1);
    }

    ntk::Pose3D depth_pose;
    depth_pose.setCameraParameters(depth_map.cols, depth_map.rows, depth_map.cols/2.f, depth_map.rows/2.f);
    depth_pose.applyTransformBefore(cv::Vec3f(0.1, 0.2, 0.3), cv::Vec3f(0.2, 0.1, 0.5));
    ntk::Pose3D rgb_pose = depth_pose;
    rgb_pose.applyTransformAfter(cv::Vec3f(-0.1,0,0), cv::Vec3f(0.05,0,0));

    cv::Point3f sum (0,0,0);

    ntk::TimeCount tc ("normal", 1);
    for (int i = 0; i < nb_it; ++i)
    {
        for_all_rc(depth_map)
        {
            float d = depth_map(r,c);
            cv::Point3f p3d = depth_pose.unprojectFromImage(cv::Point3f(c, r, d));
            p3d = rgb_pose.projectToImage(p3d);
            sum += p3d;
        }
    }
    tc.stop();
    ntk_dbg_print(sum, 1);

    vectorial::mat4f sse_mat = toSSE(depth_pose.cvInvProjectionMatrix());
    ntk::TimeCount tc_sse ("sse", 1);

    VectorialProjector depth_projector (depth_pose);
    VectorialProjector rgb_projector (rgb_pose);
    vectorial::vec4f sse_sum (0,0,0,1);
    for (int i = 0; i < nb_it; ++i)
    {
        for_all_rc(depth_map)
        {
            float d = depth_map(r,c);
            vectorial::vec4f p3d = depth_projector.unprojectFromImage(cv::Point3f(c, r, d));
            p3d = rgb_projector.projectToImage(p3d);
            sse_sum += p3d;
        }
    }

    sum = toPoint3f(sse_sum);
    tc_sse.stop();
    ntk_dbg_print(sum, 1);

    ntk::TimeCount tc_cloud ("cloud", 1);
    sum = cv::Point3f(0,0,0);
    cv::Mat4f cloud (depth_map.rows, depth_map.cols);
    cloud = cv::Vec4f(0,0,0,1);
    for (int i = 0; i < nb_it; ++i)
    {
        depth_pose.unprojectFromImage(depth_map, cv::Mat1b(), cloud);
        rgb_pose.projectToImage(cloud, cv::Mat1b(), cloud);
        for_all_rc(cloud)
        {
            sum += cv::Point3f(cloud(r,c)[0], cloud(r,c)[1], cloud(r,c)[2]);
        }
    }
    tc_cloud.stop();
    ntk_dbg_print(sum, 1);

    return 0;
}
