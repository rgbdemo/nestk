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

#ifndef NTK_DETECTION_FPFHOBJECTDETECTOR_H
#define NTK_DETECTION_FPFHOBJECTDETECTOR_H

# include "object_detector.h"
# include "feature_indexer.h"

namespace ntk
{

class VFHFeatureIndexer : public FeatureIndexer
{
public:
    struct ViewData : public ntk::XmlSerializable
    {
        ViewData() : nb_vfh_points(-1) {}
        int nb_vfh_points;

        virtual void fillXmlElement(XMLNode& element) const
        {
            setXmlAttribute(element, "nb_vfh_points", nb_vfh_points);
        }
        virtual void loadFromXmlElement(const XMLNode& element)
        {
            loadFromXmlAttribute(element, "nb_vfh_points", nb_vfh_points);
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
    VFHFeatureIndexer(const ObjectDatabase& db, LocatedFeature::FeatureType type);
    virtual ~VFHFeatureIndexer();

public:
    virtual void fillXmlElement(XMLNode& element) const;
    virtual void loadFromXmlElement(const XMLNode& element);

public:
    virtual unsigned nbPoints() const { return m_points.size(); }
    virtual unsigned nbPointsInObjectView(const VisualObjectView& obj) const;

    virtual const LocatedFeature& getPoint(int point_id) const
    { ntk_assert(point_id < (int)nbPoints(), "Invalid point id."); return *m_points[point_id]; }

    virtual std::string getIndexerName() const { return "vfh"; }

public:
    virtual MatchResults findMatches(const LocatedFeature& p) const;

protected:
    virtual void rebuild();

protected:
    std::vector<ViewData> m_objects_view_data;
    std::vector<PointData> m_points_data;
    std::vector<LocatedFeature*> m_points;
};

class VFHObjectMatch : public ObjectMatch
{
public:
    VFHObjectMatch(const ObjectDetectorData& data, const ObjectPosePtr& pose) :
        ObjectMatch(data, pose)
    {}

    VFHObjectMatch() {}

public:
    void setScore(float score) { m_score = score; }
    virtual double score() const { return m_score; }

protected:
    float m_score;
};

class VFHObjectDetector : public ObjectDetector
{
    typedef ObjectDetector super;

public:
    VFHObjectDetector();
    virtual ~VFHObjectDetector();

public:
    virtual void fillXmlElement(XMLNode& element) const {}
    virtual void loadFromXmlElement(const XMLNode& element) {}

public:
    virtual unsigned nbObjectMatches() const { return m_matches.size(); }
    virtual const ObjectMatch& objectMatch(unsigned idx) const { return m_matches[idx]; }
    virtual ObjectMatch& objectMatch(unsigned idx) { return m_matches[idx]; }
    virtual void keepOnlyBestMatch() {}

public:
    virtual void setObjectDatabase(const ObjectDatabasePtr& db);
    virtual void findObjects();

private:
    std::vector<VFHObjectMatch> m_matches;
    VFHFeatureIndexer* m_vfh_indexer;
};

} // ntk

#endif // NTK_DETECTION_FPFHOBJECTDETECTOR_H
