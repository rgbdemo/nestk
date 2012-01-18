//
// pose.cc
//
// Author: Nicolas Burrus <nicolas.burrus@ensta.fr>, (C) 2007
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
//

#include "object_pose.h"
#include "object_database.h"
#include "visual_object.h"
#include "object_detector.h"

#include <ntk/ntk.h>

using namespace ntk;

namespace ntk
{

  ObjectPose :: 
  ObjectPose (const ObjectDetectorData& data,
              const VisualObjectView& model_view,
              const ntk::AffineTransform& affine_transform)
    : m_finder_data(data),
    m_model_view (&model_view),
    m_pose_2d(affine_transform),
    m_has_projections(false)
  {    
  }

  void ObjectPose :: 
  fillXmlElement(XMLNode& element) const
  {
    addXmlChild(element, "pose2d", m_pose_2d);
    addXmlChild(element, "pose3d", m_pose_3d);
  }
  
  void ObjectPose ::
  loadFromXmlElement(const XMLNode& element)
  {
    loadFromXmlChild(element, "pose2d", m_pose_2d);
    loadFromXmlChild(element, "pose3d", m_pose_3d);
    finalize();
  }

  void ObjectPose :: finalize()
  {
    computeProjections();
  }

  void ObjectPose :: computeProjections()
  {
    if (!hasPose3D())
    {
      m_projected_bounding_rect = apply_transform(m_pose_2d.affineTransform(), modelView().boundingRect());
      cv::Rect_<float> bbox = bounding_box(m_projected_bounding_rect);
      m_max_projected_dimension = std::max(bbox.width, bbox.height);
      m_projected_topleft = apply_transform(m_pose_2d.affineTransform(), bbox.tl());
      m_projected_bottomright = apply_transform(m_pose_2d.affineTransform(), bbox.br());
      m_projected_center = apply_transform(m_pose_2d.affineTransform(), box_center(bbox));
    }
    else
    {
      m_projected_bounding_rect = project_bounding_box_to_image(pose3d(), model().boundingBox());
      cv::Rect_<float> bbox = bounding_box(m_projected_bounding_rect);
      m_max_projected_dimension = std::max(bbox.width, bbox.height);
      m_projected_topleft = bbox.tl();
      m_projected_bottomright = bbox.br();
      m_projected_center = box_center(bbox);
    }
    m_has_projections = true;
  }

  const VisualObject& ObjectPose ::
  model() const
  {
    return m_model_view->object();
  }

  const NtkDebug& operator<< (const NtkDebug& os, const ObjectPose& pose)
  {
    os << "model=" << pose.model().name();
    os << "view=" << pose.modelView().id().view_id_in_object;
    os << pose.pose2d();
    os << pose.pose3d();
    os << "\n[centroid= " << pose.pose3d().cameraTransform(cv::Point3f(0,0,0)) << "]\n";
    // FIXME: output more information.
    return os;
  }

} // end of avs
