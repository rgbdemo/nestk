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

#ifndef NTK_MESH_RGBD_MODELER_H
#define NTK_MESH_RGBD_MODELER_H

#include <ntk/core.h>
#include <ntk/camera/rgbd_image.h>
#include <ntk/mesh/mesh.h>
#include <ntk/geometry/plane.h>
#include <ntk/utils/opencv_utils.h>
#include <ntk/thread/utils.h>
#include <ntk/thread/event.h>

namespace ntk
{

class Pose3D;

class RGBDModeler
{
public:
    RGBDModeler() :
        m_global_depth_offset(0),
        m_use_surfels(false)
    {}

public:
    virtual void acquireLock() const {}
    virtual void releaseLock() const {}
    virtual const Mesh& currentMesh() const { return m_mesh; }
    const RGBDImage& lastImage() const { return m_last_image; }
    void setGlobalDepthOffset(float offset) { m_global_depth_offset = offset; }
    virtual float resolution() const { return 0; }
    virtual void setResolution(float resolution) {}
    virtual void setDepthMargin(float depth_margin) {}
    const ntk::Plane& supportPlane() const { return m_support_plane; }
    virtual int numPoints() const { return -1; }
    void setSurfelRendering(bool use_surfels) { m_use_surfels = use_surfels; }
    void setBoundingBox(const Rect3f& bbox) { m_bounding_box = bbox; }

public:
    virtual bool addNewView(const RGBDImage& image, Pose3D& depth_pose);
    virtual void computeMesh() {}
    virtual void computeSurfaceMesh() { computeMesh(); }
    virtual void computeAccurateVerticeColors() {}
    virtual void reset() { m_mesh.clear(); }

protected:
    ntk::Mesh m_mesh;
    float m_global_depth_offset;
    ntk::Plane m_support_plane;
    ntk::RGBDImage m_last_image;
    bool m_use_surfels;
    Rect3f m_bounding_box;
};
ntk_ptr_typedefs(RGBDModeler)

class DummyRGBDModeler : public RGBDModeler
{
public:
    DummyRGBDModeler(int processing_time_msecs = 20)
        : RGBDModeler()
        , m_processing_time (processing_time_msecs)
    {}

public:
    virtual void acquireLock() const {}
    virtual void releaseLock() const {}
    virtual int numPoints() const { return 0; }

public:
    virtual bool addNewView(const RGBDImage& image, Pose3D& depth_pose);

private:
    int m_processing_time;
};
ntk_ptr_typedefs(DummyRGBDModeler)

class RGBDModelerInOwnThread : public RGBDModeler, public EventProcessingBlockInOwnThread
{
public:
    struct FrameEventData : public ntk::EventData
    {
        TYPEDEF_THIS(FrameEventData)

        CLONABLE_EVENT_DATA

        enum EventType { NewImage = 0,
                         ComputeMesh = 1,
                         ComputeSurfaceMesh = 2,
                         ComputeAccurateSurfaceColor = 3,
                         Reset = 4};

        EventType type;
        RGBDImage image;
        ntk::Pose3D pose;
    };
    ntk_ptr_typedefs(FrameEventData)

    struct MeshEventSender : public ntk::EventBroadcaster
    {};

public:
    RGBDModelerInOwnThread(Name name, RGBDModelerPtr child)
    : EventProcessingBlockInOwnThread(name)
    , child (child)
    {}

    virtual ~RGBDModelerInOwnThread();

public:
    virtual void acquireLock() const { lock.lock(); }
    virtual void releaseLock() const { lock.unlock(); }
    virtual const Mesh& currentMesh() const; //  { return child->currentMesh(); }
    const RGBDImage& lastImage() const { return child->lastImage(); }
    void setGlobalDepthOffset(float offset) { child->setGlobalDepthOffset(offset); }
    virtual float resolution() const { return child->resolution(); }
    virtual void setResolution(float resolution) { return child->setResolution(resolution); }
    virtual void setDepthMargin(float depth_margin) { return setDepthMargin(depth_margin); }
    const ntk::Plane& supportPlane() const { return child->supportPlane(); }
    virtual int numPoints() const { return child->numPoints(); }
    void setSurfelRendering(bool use_surfels) { return child->setSurfelRendering(use_surfels); }
    void setBoundingBox(const Rect3f& bbox) { return child->setBoundingBox(bbox); }

public:
    virtual bool addNewView(const RGBDImage& image, Pose3D& depth_pose);
    virtual void computeMesh();
    virtual void computeSurfaceMesh();
    virtual void computeAccurateVerticeColors();
    virtual void reset();

public:
    virtual void run();

protected:
    RGBDModelerPtr child;
    Mesh new_mesh;
    MeshEventSender mesh_event_sender;
    mutable RecursiveQMutex lock;
};
ntk_ptr_typedefs(RGBDModelerInOwnThread)

} // ntk

#endif // NTK_MESH_RGBD_MODELER_H
