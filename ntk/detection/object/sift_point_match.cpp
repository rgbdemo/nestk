//
// point_match.cc
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

#include "sift_point_match.h"
#include "located_feature.h"
#include "sift_object_detector.h"
#include "feature_indexer.h"
#include "object_pose.h"
#include <ntk/ntk.h>

using namespace ntk;
using namespace cv;

namespace ntk
{

  const LocatedFeature& SiftPointMatch :: modelPoint() const
  {
    ntk_assert(m_model_point, "Null model point!!");
    return *m_model_point;
  }

  void SiftPointMatch ::
  fillXmlElement(XMLNode& element) const
  {
    setXmlAttribute(element, "id", m_id);
    setXmlAttribute(element, "model_point_id", modelPoint().idInIndexer());
    setXmlAttribute(element, "obs_point_id", obsPoint().idInIndexer());
    setXmlAttribute(element, "strength", m_strength);
    setXmlAttribute(element, "predicted", m_from_prediction);
    setXmlAttribute(element, "closest_in_db", m_closest_in_db);
    addXmlChild(element, "point_transform", m_point_transform);
  }
  
  void SiftPointMatch ::
  loadFromXmlElement(const XMLNode& element, XmlContext* context)
  {
    const SiftObjectDetector* finder = dynamic_cast<const SiftObjectDetector*>(context);
    ntk_assert(finder != 0, "Bad context.");

    m_finder_data = &finder->data();

    loadFromXmlAttribute(element, "id", m_id);
    
    int model_point_id, obs_point_id;
    loadFromXmlAttribute(element, "model_point_id", model_point_id);
    loadFromXmlAttribute(element, "obs_point_id", obs_point_id);
    m_model_point = &(finder->siftDatabase().getPoint(model_point_id));
    m_obs_point = &(finder->imageSiftPoint(obs_point_id));
    
    loadFromXmlAttribute(element, "strength", m_strength);
    loadFromXmlAttribute(element, "predicted", m_from_prediction);
    loadFromXmlAttribute(element, "closest_in_db", m_closest_in_db);
    loadFromXmlChild(element, "point_transform", m_point_transform);
    computePose();
  }

  void SiftPointMatch :: computePointTransform()
  {
    const FeatureLocation& model_loc = modelPoint().location();
    const FeatureLocation& obs_loc = obsPoint().location();

    m_point_transform.orientation = normalize_angle_0_2pi(obs_loc.orientation - model_loc.orientation);
    if (obs_loc.has_depth && model_loc.has_depth)
    {
      m_point_transform.scale = model_loc.p_image.z / obs_loc.p_image.z;
    }
    else
      m_point_transform.scale = obs_loc.scale / model_loc.scale;
    Point2f p (model_loc.p_image.x, model_loc.p_image.y);
    double cost = cos(m_point_transform.orientation);
    double sint = sin(m_point_transform.orientation);
    // must apply rotation transform.
    Point2f pmodel_after_rotation = Point2f((p.x*cost-p.y*sint) * m_point_transform.scale,
                                            (p.x*sint+p.y*cost) * m_point_transform.scale);
    m_point_transform.translation = Point2f(obs_loc.p_image.x-pmodel_after_rotation.x,
                                            obs_loc.p_image.y-pmodel_after_rotation.y);
  }
  
  void SiftPointMatch :: computePose()
  {
    const FeatureLocation& obs_loc = obsPoint().location();
    const FeatureLocation& model_loc = modelPoint().location();
    ntk::AffineTransform m;
    m = m.translated(-model_loc.p_image.x, -model_loc.p_image.y)
         .rotated(m_point_transform.orientation)
         .scaled(m_point_transform.scale, m_point_transform.scale)
         .translated(obs_loc.p_image.x, obs_loc.p_image.y);
    ObjectPose* pose = new ObjectPose(*m_finder_data, modelPoint().visualObjectView(), m);
    pose->finalize();
    m_pose = toPtr(pose);
  }
  
  const NtkDebug& operator<< (const NtkDebug& os, const SiftPointMatch& m)
  {
    if (m.fromPrediction()) os << "[Predicted]";
    if (m.closestInDb()) os << "[Closest]";
    os << " [strength=" << m.strength() << "]";
    //os << m.pointTransform();
    return os;
  }

} // end of avs
