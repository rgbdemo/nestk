//
// database_indexer.h
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

#ifndef FROL_SIFT_database_indexer_H
# define FROL_SIFT_database_indexer_H

# include <ntk/ntk.h>

# include "visual_object_view.h"
# include "object_database.h"

# include "located_feature.h"

namespace ntk
{

  class FeatureIndexer : public ntk::XmlSerializable
  {
  public:
    struct Match
    {
      Match() : point(0), score(std::numeric_limits<double>::max()) {}
      Match(const LocatedFeature* p, double d) : point(p), score(d) {}
      const LocatedFeature* point;
      double score; /* dist ratio en general */
    };

    struct MatchResults
    {
      MatchResults() {}
      std::vector<Match> matches;
    };

  public:
    FeatureIndexer(const ObjectDatabase& db, LocatedFeature::FeatureType type);
    virtual ~FeatureIndexer() {}
    virtual void loadOrBuild();

  public:
    const ObjectDatabase& objectDatabase() const { return m_db; }
    LocatedFeature::FeatureType featureType() const { return m_feature_type; }

    std::string getIndexerFilename() const;
    virtual std::string getIndexerName() const = 0;

  public:
    virtual unsigned nbPoints() const = 0;
    virtual unsigned nbPointsInObjectView (const VisualObjectView& view) const = 0;
    virtual const LocatedFeature& getPoint(int point_id) const = 0;

  public:
    virtual MatchResults findMatches(const LocatedFeature& p) const = 0;

  protected:
    virtual void rebuild() = 0;
    virtual bool loadFromDisk();
    virtual void saveToDisk() const;

  protected:
    const ObjectDatabase& m_db;
    mutable unsigned m_nb_requests;
    LocatedFeature::FeatureType m_feature_type;
    mutable RecursiveQReadWriteLock m_lock;
  };


  class SimpleFeatureIndexer : public FeatureIndexer
  {
  public:
    struct ViewData : public ntk::XmlSerializable
    {
      ViewData() : nb_sift_points(-1) {}
      int nb_sift_points;

      virtual void fillXmlElement(XMLNode& element) const
      {
        setXmlAttribute(element, "nb_sift_points", nb_sift_points);
      }
      virtual void loadFromXmlElement(const XMLNode& element)
      {
        loadFromXmlAttribute(element, "nb_sift_points", nb_sift_points);
      }
    };

    struct PointData : public ntk::XmlSerializable
    {
      PointData() {}
      VisualObjectViewId visual_object_id;

      virtual void fillXmlElement(XMLNode& element) const
      {
        setXmlAttribute(element, "object_id", visual_object_id.object_id);
        setXmlAttribute(element, "view_id_in_object", visual_object_id.view_id_in_object);
        setXmlAttribute(element, "view_id_in_database", visual_object_id.view_id_in_database);
      }
      virtual void loadFromXmlElement(const XMLNode& element)
      {
        loadFromXmlAttribute(element, "object_id", visual_object_id.object_id);
        loadFromXmlAttribute(element, "view_id_in_object", visual_object_id.view_id_in_object);
        loadFromXmlAttribute(element, "view_id_in_database", visual_object_id.view_id_in_database);
      }
    };   

  public:
    SimpleFeatureIndexer(const ObjectDatabase& db, LocatedFeature::FeatureType type);
    virtual ~SimpleFeatureIndexer();

  public:
    virtual void fillXmlElement(XMLNode& element) const;
    virtual void loadFromXmlElement(const XMLNode& element);

  public:    
    virtual unsigned nbPoints() const { return m_points.size(); }
    virtual unsigned nbPointsInObjectView (const VisualObjectView& view) const;

    virtual const LocatedFeature& getPoint(int point_id) const
    { ntk_assert(point_id < (int)nbPoints(), "Invalid point id."); return *m_points[point_id]; }

    const VisualObjectView& objectView(int point_id) const;
    VisualObjectViewId objectViewId(int point_id) const;

    int objectDataIndex(int point_index) { return objectView(point_index).id().view_id_in_database; }

    virtual std::string getIndexerName() const { return "simple"; }

  public:
    virtual MatchResults findMatches(const LocatedFeature& p) const;

  protected:
    virtual void rebuild();

  protected:
    std::vector<ViewData> m_objects_view_data;
    std::vector<PointData> m_points_data;
    std::vector<LocatedFeature*> m_points;
  };

  void compute_feature_depth(std::list<LocatedFeature*>& features,
                             const VisualObjectView& view,
                             const cv::Mat1f& depth_image);

} // end of avs

#endif // ndef FROL_SIFT_database_indexer_H
