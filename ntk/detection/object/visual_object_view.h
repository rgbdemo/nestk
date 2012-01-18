//
// Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2009
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef avs_ads_detection_VISUALOBJECTVIEW_H
#define avs_ads_detection_VISUALOBJECTVIEW_H

# include <ntk/ntk.h>
# include <ntk/geometry/pose_3d.h>
# include <ntk/camera/calibration.h>

namespace ntk
{

  class ObjectDatabase;
  class VisualObject;

  struct VisualObjectViewId
  {
    VisualObjectViewId() : object_id(-1), view_id_in_object(-1), view_id_in_database(-1) {}
    bool isValid() const { return object_id >= 0 && view_id_in_object >= 0 && view_id_in_database >= 0; }
    int object_id;
    int view_id_in_object;
    int view_id_in_database;
  };

  class VisualObjectView : public ntk::XmlSerializable
  {
   public:
     VisualObjectView(const VisualObject* object, const VisualObjectViewId& id)
       : m_object(object), m_id(id) {}

     VisualObjectView(const VisualObject* object,
                      const std::string& image_dir,
                      const VisualObjectViewId& id);

     void buildCache();

   public:
     virtual void fillXmlElement(XMLNode& element) const;
     virtual void loadFromXmlElement(const XMLNode& element, ntk::XmlContext* context);

   public:
     std::string name() const;
     const VisualObjectViewId& id() const { return m_id; }
     const VisualObject& object() const { return *m_object; }

     const std::string& imageDir() const { return m_image_dir; }
     std::string depthImageFile() const { return m_image_dir + "/mapped_depth.raw"; }
     bool hasDepth() const { return m_has_depth; }
     bool hasMask() const { return m_has_mask; }
     bool hasInnerMask() const { return m_has_inner_mask; }

     std::string maskImageFile() const { return m_image_dir + "/mask.png"; }
     std::string innerMaskImageFile() const { return m_image_dir + "/inner_mask.png"; }
     std::string colorImageFile() const { return m_image_dir + "/color.png"; }

     cv::Mat1b grayImage() const;
     cv::Mat3b colorImage() const;
     cv::Mat1b maskImage() const;
     cv::Mat1b innerMaskImage() const;
     cv::Mat1f mappedDepthImage() const;

     const std::vector<cv::Point2f>& depthContour() const { return m_depth_edge_contour; }
     const std::vector<cv::Point3f>& depthSamples() const { return m_depth_samples; }

     void transformPose(const cv::Vec3f& tranlation, const cv::Vec3f& rotation);
     const ntk::Pose3D& objectPose() const { return m_object_pose; }

     const cv::Rect& boundingRect() const { return m_bounding_rect; }
     int imageWidth() const { return m_image_width; }
     int imageHeight() const { return m_image_height; }

   protected:
     void buildFromModel();
     void buildFromPrecomputed();
     void buildRGBDImageCache();

   private:
     const VisualObject* m_object;
     VisualObjectViewId m_id;
     std::string m_image_dir;
     ntk::Pose3D m_object_pose;
     cv::Rect m_bounding_rect;
     std::vector<cv::Point2f> m_depth_edge_contour;
     std::vector<cv::Point3f> m_depth_samples;
     int m_image_width;
     int m_image_height;
     bool m_has_depth;
     bool m_has_mask;
     bool m_has_inner_mask;
  };

}

#endif // avs_ads_detection_VISUALOBJECTVIEW_H
