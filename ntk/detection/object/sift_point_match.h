//
// point_match.h
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

#ifndef FROL_sift_point_match_H
# define FROL_sift_point_match_H

# include <ntk/ntk.h>
# include "located_feature.h"
# include <stdint.h>

namespace ntk
{

  class LocatedFeature;
  class VisualObject;
  class ObjectPose; ntk_ptr_typedefs(ObjectPose);
  class ObjectDetectorData;

  struct SiftPointTransform : public ntk::XmlSerializable
  {
    SiftPointTransform() : orientation(0), scale(1), translation(0,0)
    {}

    SiftPointTransform(double orientation, double scale, const cv::Point2f& translation)
        : orientation(orientation), scale(scale), translation(translation)
    {}

    virtual void fillXmlElement(XMLNode& element) const
    {
      setXmlAttribute(element, "orientation", orientation);
      setXmlAttribute(element, "scale", scale);
      setXmlAttribute(element, "translation", translation);
    }

    virtual void loadFromXmlElement(const XMLNode& element)
    {
      loadFromXmlAttribute(element, "orientation", orientation);
      loadFromXmlAttribute(element, "scale", scale);
      loadFromXmlAttribute(element, "translation", translation);
    }

    double orientation;
    double scale;
    cv::Point2f translation;
  };

  class SiftPointMatch : public ntk::XmlSerializable
  {
    public:
      SiftPointMatch(const ObjectDetectorData& data,
                     const LocatedFeature& obs_point, const LocatedFeature& model_point,
                     double strength, int id)
          : m_finder_data(&data),
            m_obs_point(&obs_point), m_model_point(&model_point),
            m_strength(strength), m_geometric_error(0),
            m_from_prediction(false), m_closest_in_db(false),
            m_timestamp(-1), m_id(id), distance_ratio(0)
      {
        computePointTransform();
        computePose();
      }

      SiftPointMatch()
        : m_finder_data(0), m_obs_point(0), m_model_point(0), m_id(-1), distance_ratio(0)
      {}

    public:
      virtual void fillXmlElement(XMLNode& element) const;
      virtual void loadFromXmlElement(const XMLNode& element, ntk::XmlContext* context);

    public:
      int id() const { return m_id; }

      bool fromPrediction() const { return m_from_prediction; }
      void setFromPrediction(bool b) { m_from_prediction = b; }

      void setClosestInDb(bool b) { m_closest_in_db = b; }
      bool closestInDb() const { return m_closest_in_db; }

      const VisualObjectView& modelView() const { return modelPoint().visualObjectView(); }
      const LocatedFeature& obsPoint() const { return *m_obs_point; }
      // const LocatedFeature& modelPoint() const { return *m_model_point; }
      const LocatedFeature& modelPoint() const;
      const SiftPointTransform& pointTransform() const { return m_point_transform; }
      const ObjectPoseConstPtr& pose() const { return m_pose; }

      double strength() const { return m_strength; }
      double geometricError() const { return m_geometric_error; }
      void setGeometricError(double e) const { m_geometric_error = e; }

      void setTimestamp(int64_t t) const { m_timestamp = t; }
      int64_t timestamp() const { return m_timestamp; }

    private:
      void computePointTransform();
      void computePose();

    private:
      const ObjectDetectorData* m_finder_data;
      const LocatedFeature* m_obs_point;
      const LocatedFeature* m_model_point;
      SiftPointTransform m_point_transform;
      ObjectPoseConstPtr m_pose;
      double m_strength;
      // not very elegant, but this is set during geoemtric optimization.
      mutable double m_geometric_error;
      bool m_from_prediction;
      bool m_closest_in_db;
      mutable int64_t m_timestamp;
      int m_id;

    public:
      double distance_ratio;
  };
  ntk_ptr_typedefs(SiftPointMatch);

  const NtkDebug& operator<< (const NtkDebug& os, const SiftPointMatch& m);

  struct SiftPointMatchConstPtrLt
  {
    bool operator()(const SiftPointMatchConstPtr& lhs, const SiftPointMatchConstPtr& rhs) const
    {
      if (lhs->strength() < rhs->strength()) return true;
      else if (lhs->strength() > rhs->strength()) return false;
      return lhs < rhs;
    }
  };
  typedef std::set<SiftPointMatchConstPtr, SiftPointMatchConstPtrLt> SiftPointMatchConstPtrSet;
  typedef std::set<SiftPointMatchPtr, SiftPointMatchConstPtrLt> SiftPointMatchPtrSet;

} // end of avs

#endif // ndef FROL_sift_point_match_H
