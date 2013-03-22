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

#ifndef FROL_common_pose_H
# define FROL_common_pose_H

# include <ntk/ntk.h>
# include <ntk/geometry/pose_3d.h>
# include <ntk/geometry/polygon.h>
# include "pose_2d.h"

namespace ntk
{

  class VisualObject;
  class VisualObjectView;
  class ObjectDetectorData;

  // Represent model location, rotation and scale in the new image.
  class ObjectPose : public ntk::XmlSerializable
  {
    public:
      ObjectPose(const ObjectDetectorData& data,
                 const VisualObjectView& model_view,
                 const ntk::AffineTransform& affine_transform);

    public:
      virtual void fillXmlElement(XMLNode& element) const;
      virtual void loadFromXmlElement(const XMLNode& element);

    public:
      const VisualObject& model() const;
      const VisualObjectView& modelView() const { return *m_model_view; }

      void setPose2D(const Pose2D& pose2d) { m_pose_2d = pose2d; }
      void setPose3D(const ntk::Pose3D& pose3d) { m_pose_3d = pose3d; }
      const Pose2D& pose2d() const { return m_pose_2d; }
      const ntk::Pose3D& pose3d() const { return m_pose_3d; }
      bool hasPose3D() const { return m_pose_3d.isValid(); }

      const ObjectDetectorData& finderData() const { return m_finder_data; }
      void setAffineTransform(const ntk::AffineTransform& affine_transform);

      void ensureHasProjections() const { ntk_assert(m_has_projections, "Projections not computed"); }
      bool hasProjections() const { return m_has_projections; }

      const ntk::Polygon2d& projectedBoundingRect() const
      { ensureHasProjections();return m_projected_bounding_rect; }
      const cv::Point2f& projectedTopLeft() const { ensureHasProjections(); return m_projected_topleft; }
      const cv::Point2f& projectedCenter() const { ensureHasProjections(); return m_projected_center; }
      const cv::Point2f& projectedBottomRight() const { ensureHasProjections(); return m_projected_bottomright; }
      double maxProjectedDimension() const { ensureHasProjections(); return m_max_projected_dimension; }

      void computeProjections();

      // Compute projections, etc.
      void finalize();
      
    private:
      const ObjectDetectorData& m_finder_data;
      const VisualObjectView* m_model_view;
      ntk::Pose3D m_pose_3d;
      Pose2D m_pose_2d;
      bool m_has_pose_3d;
      ntk::Polygon2d m_projected_bounding_rect;
      cv::Point2f m_projected_topleft;
      cv::Point2f m_projected_bottomright;
      cv::Point2f m_projected_center;
      double m_max_projected_dimension;
      bool m_has_projections;
  };
  ntk_ptr_typedefs(ObjectPose);

  const NtkDebug& operator<< (const NtkDebug& os, const ObjectPose& pose);

} // end of avs

#endif // ndef FROL_common_pose_H
