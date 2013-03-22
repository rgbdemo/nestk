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

#include "feature_indexer.h"
#include "located_feature.h"
#include "object_database.h"
#include "visual_object.h"

#include <QTime>
#include <QDebug>

#include <ntk/ntk.h>

using namespace ntk;

namespace ntk
{

  FeatureIndexer :: FeatureIndexer(const ObjectDatabase& db, LocatedFeature::FeatureType type)
    : m_db(db), m_feature_type(type)
  {
  }

  std::string FeatureIndexer :: getIndexerFilename() const
  {
    return std::string("avs_metadata/feature_indexer_")
           + getIndexerName()
           + "_" + LocatedFeature::featureTypeName(m_feature_type)
           + ".xml";
  }

  bool FeatureIndexer :: loadFromDisk()
  {
    QFileInfo f ((m_db.directory() + "/" + getIndexerFilename()).c_str());
    if (!f.isFile()) return false;
    try { loadFromXml(f); }
    catch (const std::exception& e)
    {
      ntk_log() << "Exception received: " << e.what();
      return false;
    }
    return true;
  }

  void FeatureIndexer :: saveToDisk() const
  {
    QFileInfo f ((m_db.directory() + "/" + getIndexerFilename()).c_str());
    saveAsXml(f);
  }

  void FeatureIndexer :: loadOrBuild()
  {
    if (!loadFromDisk())
    {
      ntk_dbg(1) << "Could not load simple sift database indexed, rebuilding.";
      rebuild();
      saveToDisk();
    }
    else
    {
      ntk_dbg(1) << "FeatureIndexer reloaded from: "
          << m_db.directory() + "/" + getIndexerFilename();
    }
  }

} // end of avs

namespace ntk
{

  //FIXME: temp!!!
  int nb_filtered = 0;
  int nb_analyzed = 0;
  QReadWriteLock lock;


  SimpleFeatureIndexer :: SimpleFeatureIndexer(const ObjectDatabase& db,
                                                         LocatedFeature::FeatureType type)
    : FeatureIndexer(db, type)
  {
  }

  SimpleFeatureIndexer :: ~SimpleFeatureIndexer()
  {
    ntk_dbg_print(nb_analyzed, 0);
    ntk_dbg_print(nb_filtered, 0);
    foreach_idx(i, m_points)
        delete m_points[i];
  }

  void SimpleFeatureIndexer :: fillXmlElement(XMLNode& element) const
  {
    setXmlAttribute(element, "database_id", m_db.uniqueId());
    int nb_objects = objectDatabase().nbVisualObjectsViews();
    ntk_assert(nb_objects == (int)m_objects_view_data.size(), "Corrupted indexer.");

    setXmlAttribute(element, "nb_objects", nb_objects);
    setXmlAttribute(element, "nb_sift_points", nbPoints());

    foreach_idx(i, m_objects_view_data)
    {
      XMLNode child = addXmlChildAtBeginning(element, "object_data", m_objects_view_data[i]);
      setXmlAttribute(child, "id", (int)i);
    }

    foreach_idx(i, m_points)
    {
      XMLNode child = addXmlChildAtBeginning(element, "sift_point_data", m_points_data[i]);
      setXmlAttribute(child, "id", m_points[i]->idInIndexer());
      addXmlChild(child, "raw_data", *m_points[i]);
    }
  }

  void SimpleFeatureIndexer :: loadFromXmlElement(const XMLNode& element)
  {
    int database_id;
    loadFromXmlAttribute(element, "database_id", database_id);
    if (database_id != objectDatabase().uniqueId())
      ntk_throw_exception("Index does not correspond to database id.");

    int nb_objects; loadFromXmlAttribute(element, "nb_objects", nb_objects);
    if (nb_objects != (int)objectDatabase().nbVisualObjectsViews())
      ntk_throw_exception("Sift index does not have the same number of objects than database");

    int nb_sift_points; loadFromXmlAttribute(element, "nb_sift_points", nb_sift_points);
    if (nb_sift_points < 0)
      ntk_throw_exception("nb_sift_points cannot be negative");

    m_objects_view_data.clear(); m_objects_view_data.resize(nb_objects);
    m_points_data.clear(); m_points_data.resize(nb_sift_points);
    m_points.clear(); m_points.resize(nb_sift_points, 0);

    for (int i = 0; i < element.nChildNode(); ++i)
    {
      XMLNode e = element.getChildNode(i);
      if (e.getNameAsString() == "object_data")
      {
        int id; loadFromXmlAttribute(e, "id", id);
        ntk_assert(m_objects_view_data[id].nb_sift_points == -1, "Multiple object data");
        m_objects_view_data[id].loadFromXmlElement(e);
      }

      if (e.getNameAsString() == "sift_point_data")
      {
        LocatedFeature* point = new LocatedFeature();
        int id_in_indexer; loadFromXmlAttribute(e, "id", id_in_indexer);
        ntk_assert(id_in_indexer >= 0 && id_in_indexer < nb_sift_points, "Bad point id.");
        ntk_assert(!m_points_data[id_in_indexer].visual_object_id.isValid(), "Multiple point data");

        m_points_data[id_in_indexer].loadFromXmlElement(e);
        loadFromXmlChild(e, "raw_data", *point);
        point->setIdInIndexer(id_in_indexer);
        point->setVisualObjectView(objectDatabase().visualObjectView(m_points_data[id_in_indexer].visual_object_id));
        m_points[id_in_indexer] = point;
      }
    }

    // Post conditions and finalization.

    foreach_idx(i, m_objects_view_data)
        ntk_assert(m_objects_view_data[i].nb_sift_points >= 0, "Object data missing.");

    foreach_idx(i, m_points_data)
        ntk_assert(m_points_data[i].visual_object_id.isValid(), "Point data missing.");

    foreach_idx(i, m_points)
        ntk_assert(m_points[i] != 0, "Sift point missing.");
  }

  unsigned SimpleFeatureIndexer :: nbPointsInObjectView(const VisualObjectView& obj) const
  {
    ntk_assert(obj.id().view_id_in_database < m_objects_view_data.size(), "Outbound access.");
    return m_objects_view_data[obj.id().view_id_in_database].nb_sift_points;
  }

  VisualObjectViewId SimpleFeatureIndexer :: objectViewId(int point_id) const
  {
    return m_points_data[point_id].visual_object_id;
  }

  const VisualObjectView& SimpleFeatureIndexer ::
      objectView(int point_id) const
  {
    VisualObjectViewId object_view_id = objectViewId(point_id);
    return objectDatabase().visualObjectView(object_view_id);
  }


  void compute_feature_depth(std::list<LocatedFeature*>& features,
                             const VisualObjectView& view,
                             const cv::Mat1f& depth_image)
  {
    ntk_ensure(depth_image.data, "Could not load depth image");
    ntk_dbg_print(view.name(), 1);
    foreach_const_it(it, features, std::list<LocatedFeature*>)
    {
      double x = (*it)->location().p_image.x;
      double y = (*it)->location().p_image.y;
      double depth = depth_image(y,x);
      if (flt_eq(depth, 0, 1e-5))
        continue;
      (*it)->locationRef().p_world
          = view.objectPose().unprojectFromImage(cv::Point2f(x,y), depth);
      (*it)->locationRef().setDepth(depth);
    }
  }

  void SimpleFeatureIndexer :: rebuild()
  {
    m_points.clear();
    m_points_data.clear();
    m_objects_view_data.resize(m_db.nbVisualObjectsViews());
    for (unsigned i = 0; i < m_db.nbVisualObjects(); ++i)
    {
      const VisualObject& obj = m_db.visualObject(i);
      foreach_idx(j, obj.views())
      {
        ntk_dbg(1) << "Build indexer for view " << j;
        const VisualObjectView& view = obj.views()[j];
        std::list<LocatedFeature*> output;
        RGBDImage image;
        image.rgbRef() = view.colorImage();
        image.rgbAsGrayRef() = view.grayImage();
        image.depthRef() = view.mappedDepthImage();
        compute_feature_points(output, image, m_feature_type);
        if (view.hasDepth())
          compute_feature_depth(output, view, image.depth());
        foreach_it(it, output, std::list<LocatedFeature*>)
        {          
          (*it)->setIdInIndexer(m_points.size());
          (*it)->setVisualObjectView(view);
          m_points.push_back(*it);
          PointData point_data; point_data.visual_object_id = view.id();
          m_points_data.push_back(point_data);
        }
        ViewData d; d.nb_sift_points = output.size();
        m_objects_view_data[view.id().view_id_in_database] = d;
      }
    }
  }

  SimpleFeatureIndexer::MatchResults
  SimpleFeatureIndexer :: findMatches(const LocatedFeature& p) const
  {
    QReadLocker locker(&m_lock);
    float min_dist = std::numeric_limits<float>::max();
    float min_dist2 = std::numeric_limits<float>::max();

    std::vector<LocatedFeature*>::const_iterator closest_it = m_points.end();
    std::vector<LocatedFeature*>::const_iterator closest_it2 = m_points.end();
    foreach_const_it(it, m_points, std::vector<LocatedFeature*>)
    {
      if (!(*it)->visualObjectView().object().isEnabled()) continue;
      if (!(*it)->isEnabled()) continue;

      float d = 0;    

      if (p.isSurf() && ((**it).location().laplacian != p.location().laplacian))
      {
        d = std::numeric_limits<float>::max();
      }
      else
        d = euclidian_distance((**it).descriptor(), p.descriptor(), min_dist2);
      // unsigned d = chi2_distance(**it, p, min_dist2);
      // unsigned d = emd_distance(**it, p, min_dist2);
      if (d < min_dist)
      {
        min_dist2 = min_dist;
        closest_it2 = closest_it;
        min_dist = d;
        closest_it = it;
      }
      else if (d < min_dist2)
      {
        min_dist2 = d;
        closest_it2 = it;
      }
    }
    ntk_assert(closest_it != m_points.end(), "Empty db?");
    ntk_assert(closest_it2 != m_points.end(), "Only one point in db?");
    ntk_assert(min_dist < std::numeric_limits<unsigned>::max(), "min_dist not updated.");

    const LocatedFeature* p1 = 0;
    const LocatedFeature* p2 = 0;
    if (closest_it != m_points.end()) p1 = *closest_it;
    if (closest_it2 != m_points.end()) p2 = *closest_it2;

    MatchResults c;
    if (p1 && p2)
    {
      QWriteLocker locker(&lock);
      ++nb_analyzed;

      if (0 //FIXME: disabled
          && p.location().has_depth
          && p1->location().has_depth
          && p1->visualObjectView().object().hasZCorrection())
      {
        double zm_mul_si_div_sm = p1->location().p_image.z * (p.location().scale / p1->location().scale);
        double ideal_zi = p1->visualObjectView().object().getCorrectedIdealZ(zm_mul_si_div_sm);
        double diff = (ideal_zi / p.location().p_image.z);
        if (diff < 1) diff = 1.0/diff;
        if (diff > 2.6) // scale does not match
        {
          ++nb_filtered;
          ntk_dbg_print(diff, 2);
          ntk_dbg_print(zm_mul_si_div_sm, 2);
          ntk_dbg_print(ideal_zi, 2);
          ntk_dbg_print(p.location().p_image.z, 2);
          return c;
        }
      }

      // ntk_assert(p1->visualObjectView().id().view_id_in_database < 80, "Bad id !");
      c.matches.push_back(Match(p1, min_dist/min_dist2));
    }
    return c;
  }

} // end of avs
