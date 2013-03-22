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

#ifndef FROL_common_object_match_H
# define FROL_common_object_match_H

# include <ntk/ntk.h>
# include "object_pose.h"

namespace ntk
{

  class VisualObject;
  class VisualObjectView;

  class ObjectDetectorData;

  class ObjectMatch : public ntk::XmlSerializable
  {
  public:
    ObjectMatch(const ObjectDetectorData& data, const ObjectPosePtr& pose)
      : m_finder_data(&data), m_pose(pose), m_is_correct(false), m_timestamp(-1)
    {}

    ObjectMatch() : m_finder_data(0), m_is_correct(false)
    {}
      
    virtual ~ObjectMatch () {}

  public:
    bool isCorrect() const { return m_is_correct; }
    void setCorrect(bool is_correct) { m_is_correct = is_correct; }
    
  public:    
    virtual void fillXmlElement(XMLNode& element) const;
    virtual void loadFromXmlElement(const XMLNode& element, ntk::XmlContext* context);
    virtual double score() const = 0;

  public:
    const ObjectDetectorData& finderData() const { return *m_finder_data; }
    ObjectPoseConstPtr pose() const { return m_pose; }
    ObjectPosePtr pose() { return m_pose; }
    const VisualObject& model() const { return pose()->model(); }
    const VisualObjectView& modelView() const { return pose()->modelView(); }
    const ntk::AffineTransform& affineTransform() const { return pose()->pose2d().affineTransform(); }
    const ntk::Polygon2d& projectedBoundingRect() const { return pose()->projectedBoundingRect(); }
    // const ntk::Polygon2d& projectedBoundingRect() const { return pose()->projectedBoundingRect(); }
    ntk::Polygon2d projectedBoundingBox() const;
    int timestamp() const { return m_timestamp; }
    void setTimestamp(int t) { m_timestamp = t; }
    const std::vector<cv::Point3f>& matchedPoints() const { return m_matched_points; }
    void setMatchedPoints(const std::vector<cv::Point3f>& points) { m_matched_points = points; }

  protected:
    const ObjectDetectorData* m_finder_data;
    ObjectPosePtr m_pose;
    bool m_is_correct;
    int m_timestamp;
    std::vector<cv::Point3f> m_matched_points;
  };

} // end of avs

#endif // ndef FROL_common_object_match_H
