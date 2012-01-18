//
// sift.h
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

#ifndef FROL_sift_point_H
# define FROL_sift_point_H

# include <ntk/ntk.h>
# include <ntk/camera/calibration.h>
# include "feature.h"

namespace ntk
{

  class VisualObjectView;

  class LocatedFeature : public ntk::XmlSerializable
  {
  public:
    enum FeatureType { Feature_SIFT, Feature_SURF64, Feature_SURF128, Feature_FAST, Feature_CVFH, Feature_VFH };
    static std::string featureTypeName(FeatureType type);
    static FeatureType featureTypeFromName(const std::string& name);

    public:
      LocatedFeature() : m_visual_object_view(0), m_descriptor(0),
        m_id_in_indexer(-1), m_shared_descriptor(false), m_enabled(true),
        m_feature_type(Feature_SIFT) {}

      LocatedFeature(const FeatureLocation& location,
                     FeatureDescriptorConstPtr descriptor,
                     FeatureType type = Feature_SIFT) :
          m_visual_object_view(0), m_location(location), m_descriptor(descriptor),
          m_id_in_indexer(-1), m_shared_descriptor(false), m_enabled(true), m_feature_type(type)
      {}

      ~LocatedFeature();

    private:
      LocatedFeature(const LocatedFeature& rhs);

    public:
      virtual void fillXmlElement(XMLNode& element) const;
      using ntk::XmlSerializable::loadFromXmlElement;
      virtual void loadFromXmlElement(const XMLNode& element);

    public:
      bool isSurf() const { return (m_feature_type == Feature_SURF64) || (m_feature_type == Feature_SURF128); }
      FeatureType featureType() const { return (FeatureType) m_feature_type; }
      void setFeatureType(FeatureType type) { m_feature_type = type; }

      FeatureDescriptor::Type descriptorType() const;
      int descriptorSize() const;

      static FeatureDescriptor::Type descriptorType(FeatureType type);
      static int descriptorSize(FeatureType type);

      void setEnabled(bool b) const { m_enabled = b; }
      bool isEnabled() const { return m_enabled; }

      void setIdInIndexer(int id) { m_id_in_indexer = id; }
      int idInIndexer() const { return m_id_in_indexer; }

      void setVisualObjectView(const VisualObjectView& object) { m_visual_object_view = &object; }
      bool hasVisualObjectView() const { return m_visual_object_view != 0; }
      const VisualObjectView& visualObjectView() const { return *m_visual_object_view; }

      bool useFloatDescriptors() const { return m_descriptor->hasFloatDesc(); }
      bool useByteDescriptors() const { return m_descriptor->hasByteDesc(); }
      void setSharedDescriptor(bool b) { m_shared_descriptor = b; }
      bool hasSharedDescriptor() const { return m_shared_descriptor; }

      void setDescriptor(FeatureDescriptorConstPtr d) { m_descriptor = d; }
      const FeatureDescriptor& descriptor() const { return *m_descriptor; }
      const std::vector<unsigned char>& byteDescriptors() const { return m_descriptor->byteDesc(); }
      const std::vector<float>& floatDescriptors() const { return m_descriptor->floatDesc(); }
      const FeatureLocation& location() const { return m_location; }
      cv::Point2f imagePos() const { return cv::Point2f(m_location.p_image.x, m_location.p_image.y); }
      FeatureLocation& locationRef() { return m_location; }

    public:
      bool comparableTo(const LocatedFeature& p2) const
      { return p2.byteDescriptors().size() == byteDescriptors().size(); }

    private:
      const VisualObjectView* m_visual_object_view;
      FeatureLocation m_location;
      FeatureDescriptorConstPtr m_descriptor;
      int m_id_in_indexer;
      bool m_shared_descriptor;
      mutable bool m_enabled;
      char m_feature_type;
  };

  unsigned chi2_distance(const LocatedFeature& p1, const LocatedFeature& p2,
                         unsigned stop_if_greater_than = std::numeric_limits<unsigned>::max());

  float euclidian_distance(const FeatureDescriptor& p1, const FeatureDescriptor& p2,
                           float stop_if_greater_than = std::numeric_limits<float>::max());

  unsigned emd_distance(const LocatedFeature& p1, const LocatedFeature& p2,
                        unsigned stop_if_greater_than = std::numeric_limits<unsigned>::max());

  double entropy(const LocatedFeature& p);

  void compute_siftgpu_points(std::list<LocatedFeature*>& output, const cv::Mat1b& im);
  void compute_siftgpu_client_points(std::list<LocatedFeature*>& output, const cv::Mat1b& im);
  void compute_siftpp_points(std::list<LocatedFeature*>& output, const cv::Mat1b& im, const cv::Mat1f& depth_image);
  void compute_surf_points(std::list<LocatedFeature*>& output, const cv::Mat1b& im, bool extended);
  void compute_fast_points(std::list<LocatedFeature*>& output, const RGBDImage& im);

  void compute_feature_points(std::list<LocatedFeature*>& output,
                              const ntk::RGBDImage& im,
                              LocatedFeature::FeatureType type);
  void compute_feature_points(std::list<LocatedFeature*>& output,
                              const char* filename,
                              LocatedFeature::FeatureType type,
                              const char* mask_filename = 0,
                              const cv::Mat1f& depth_image = cv::Mat1f());
  void compute_feature_points(std::list<LocatedFeature*>& output,
                              const cv::Mat1b& im,
                              LocatedFeature::FeatureType type,
                              const cv::Mat1f& depth_image = cv::Mat1f());
  void compute_feature_points(std::list<LocatedFeature*>& output, const VisualObjectView& object, LocatedFeature::FeatureType type);

} // end of avs

inline const NtkDebug& operator<<(const NtkDebug& d, const ntk::LocatedFeature& rhs)
{ d << "not implemented."; return d; }

#endif      /* !FROL_sift_point_H */
