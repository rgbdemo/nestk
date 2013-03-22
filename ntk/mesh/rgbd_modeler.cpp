
#include "rgbd_modeler.h"
#include <ntk/utils/opencv_utils.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/numeric/levenberg_marquart_minimizer.h>
#include <ntk/utils/time.h>

using namespace cv;

namespace ntk
{

bool RGBDModeler :: addNewView(const RGBDImage& image, Pose3D& depth_pose)
{
    Pose3D rgb_pose = depth_pose;
    rgb_pose.toRightCamera(image.calibration()->rgb_intrinsics, image.calibration()->R, image.calibration()->T);

    Pose3D world_to_camera_normal_pose;
    world_to_camera_normal_pose.applyTransformBefore(Vec3f(0,0,0), depth_pose.cvEulerRotation());
    Pose3D camera_to_world_normal_pose = world_to_camera_normal_pose; camera_to_world_normal_pose.invert();

    const Mat1f& depth_im = image.depth();    
    for_all_rc(depth_im)
    {
        if (!image.depthMask()(r,c))
            continue;
        float depth = depth_im(r,c) + m_global_depth_offset;
        Point3f p3d = depth_pose.unprojectFromImage(Point2f(c,r), depth);
        Point3f p_rgb = rgb_pose.projectToImage(p3d);
        if (!is_yx_in_range(image.rgb(), p_rgb.y, p_rgb.x))
            continue;

        if (!m_bounding_box.isEmpty() && !m_bounding_box.isPointInside(p3d))
            continue;

        Vec3f camera_normal = image.normal().data ? image.normal()(r, c) : Vec3f(0,0,1);
        Vec3f world_normal = camera_to_world_normal_pose.cameraTransform(camera_normal);
        normalize(world_normal);

        cv::Vec3b color = bgr_to_rgb(image.rgb()(p_rgb.y, p_rgb.x));
        m_mesh.vertices.push_back(p3d);
        m_mesh.colors.push_back(color);
        m_mesh.normals.push_back(world_normal);
    }

    return true;
}

RGBDModelerInOwnThread::~RGBDModelerInOwnThread()
{
    Q_ASSERT(!isRunning());
    //setThreadShouldExit();
    //wait();
}

bool RGBDModelerInOwnThread::addNewView(const RGBDImage &image, Pose3D &depth_pose)
{
    ntk::TimeCount tc("INOWNTHREAD", 2);
    FrameEventDataPtr data (new FrameEventData);
    data->type = FrameEventData::NewImage;
    image.copyTo(data->image);
    data->pose = depth_pose;
    newEvent(this, data);
    tc.stop();
    return true;
}

const Mesh& RGBDModelerInOwnThread::currentMesh() const
{
    // The lock has been acquired from outside.
    acquireLock();

    // FIXME: this check is a bit hacky. The mesh could have changed
    // without changing the number of vertices. But for now it works
    // well enough to avoid copying the mesh on every access.
    if (m_mesh.vertices.size() != new_mesh.vertices.size())
        *(const_cast<Mesh*>(&m_mesh)) = new_mesh;

    // Lock released.
    releaseLock();
    return m_mesh;
}

void RGBDModelerInOwnThread::computeMesh()
{
    FrameEventDataPtr data (new FrameEventData);
    data->type = FrameEventData::ComputeMesh;
    newEvent(&mesh_event_sender, data);
}

void RGBDModelerInOwnThread::computeSurfaceMesh()
{
    FrameEventDataPtr data (new FrameEventData);
    data->type = FrameEventData::ComputeSurfaceMesh;
    newEvent(&mesh_event_sender, data);
}

void RGBDModelerInOwnThread::computeAccurateVerticeColors()
{
    FrameEventDataPtr data (new FrameEventData);
    data->type = FrameEventData::ComputeAccurateSurfaceColor;
    newEvent(&mesh_event_sender, data);
}

void RGBDModelerInOwnThread::reset()
{
    FrameEventDataPtr data (new FrameEventData);
    data->type = FrameEventData::Reset;
    newEvent(&mesh_event_sender, data);
}

void RGBDModelerInOwnThread::run()
{
    setThreadShouldExit(false);

    while (!threadShouldExit())
    {
        EventListener::Event event = waitForNewEvent();
        if (event.isNull())
            continue;

        FrameEventDataPtr data = dynamic_Ptr_cast<FrameEventData>(event.data);
        ntk_assert(data, "No data!");

        switch (data->type)
        {
        case FrameEventData::NewImage:
        {
            child->addNewView(data->image, data->pose);
            break;
        }

        case FrameEventData::ComputeMesh:
        {
            child->computeMesh();
            acquireLock();
            new_mesh = child->currentMesh();
            releaseLock();
            break;
        }

        case FrameEventData::ComputeSurfaceMesh:
        {
            child->computeSurfaceMesh();
            acquireLock();
            new_mesh = child->currentMesh();
            releaseLock();
            break;
        }

        case FrameEventData::ComputeAccurateSurfaceColor:
        {
            child->computeAccurateVerticeColors();
            acquireLock();
            new_mesh = child->currentMesh();
            releaseLock();
            break;
        }

        case FrameEventData::Reset:
        {
            child->reset();
            acquireLock();
            new_mesh = child->currentMesh();
            releaseLock();
            break;
        }
        };

        reportNewEventProcessed();
    }
}

bool DummyRGBDModeler::addNewView(const RGBDImage &image, Pose3D &depth_pose)
{
    if (m_processing_time > 0)
        ntk::sleep (m_processing_time);
    return true;
}

} // ntk
