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

#include "object_match.h"
#include "object_database.h"
#include "visual_object.h"
#include "object_detector.h"

#include <ntk/utils/debug.h>

using namespace ntk;

namespace ntk
{

  void ObjectMatch :: fillXmlElement(XMLNode& element) const
  {
    setXmlAttribute(element, "model", model().name());
    setXmlAttribute(element, "view", modelView().id().view_id_in_object);
    addXmlChild(element, "pose", *m_pose);
  }
  
  void ObjectMatch :: loadFromXmlElement(const XMLNode& element, XmlContext* context)
  {
    const ObjectDetector* finder = dynamic_cast<const ObjectDetector*>(context);    
    ntk_assert(finder != 0, "Bad context.");
    m_finder_data = &finder->data();
    std::string model_name;
    loadFromXmlAttribute(element, "model", model_name);
    const VisualObject* obj = finder->objectDatabase().visualObject(model_name);
    if (obj == 0) ntk_throw_exception("Unknown model: " + model_name);

    int view_id = 0;
    loadFromXmlAttribute(element, "view", view_id);
    
    ObjectPosePtr p (new ObjectPose(*m_finder_data, obj->views()[view_id], ntk::AffineTransform::identity));
    loadFromXmlChild(element, "pose", *p);
    m_pose = p;
  }

  ntk::Polygon2d ObjectMatch :: projectedBoundingBox() const
  {
    const ntk::Rect3f& bbox = model().boundingBox();
    const Pose3D& pose3d = pose()->pose3d();
    return project_bounding_box_to_image(pose3d, bbox);
  }
  
} // end of avs
