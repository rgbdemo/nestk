//
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

#include "vfh_object_detector.h"
#include <ntk/detection/table_object_detector.h>
#include <ntk/mesh/table_object_rgbd_modeler.h>

#include <pcl/features/vfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

namespace ntk
{

static void compute_vfh_features(LocatedFeature& feature,
                                  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs,
                                  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr object_cloud)
{
    const int descriptor_size = 308;
    const float grid_size = 0.003;

    ntk_dbg_print(object_cloud->points.size(), 1);

    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setLeafSize (grid_size, grid_size, grid_size);
    grid.setInputCloud(object_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_object_cloud (new pcl::PointCloud<pcl::PointXYZ>());
    grid.filter(*filtered_object_cloud);

    ntk_dbg_print(filtered_object_cloud->points.size(), 1);

    pcl::PointCloud<pcl::Normal>::Ptr object_normals (new pcl::PointCloud<pcl::Normal> ());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (filtered_object_cloud);
#ifdef HAVE_PCL_GREATER_THAN_1_2_0
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
#else
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
#endif
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (0.01);
    ne.compute (*object_normals);

    ntk_dbg_print(object_normals->points.size(), 1);

    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud (filtered_object_cloud);
    vfh.setInputNormals (object_normals);
    vfh.setSearchMethod (tree);
    vfh.compute (*vfhs);

    ntk_dbg_print(vfhs->points.size(), 1);
    ntk_assert(vfhs->points.size() == 1, "Should be a single signature!");

    feature.setFeatureType(LocatedFeature::Feature_VFH);
    FeatureDescriptor* descriptor = new FeatureDescriptor(FeatureDescriptor::FloatDescriptor, descriptor_size);
    descriptor->floatDescRef().resize(descriptor_size);
    std::copy(vfhs->points[0].histogram,
              vfhs->points[0].histogram+descriptor_size,
              descriptor->floatDescRef().begin());
    feature.setDescriptor(toPtr(descriptor));

    pcl::io::savePCDFileASCII("debug_model.pcd", *filtered_object_cloud);
    pcl::io::savePCDFileASCII("debug_model_signatures.pcd", *vfhs);
}

}

namespace ntk
{

VFHObjectDetector :: VFHObjectDetector() : m_vfh_indexer(0)
{

}

VFHObjectDetector :: ~VFHObjectDetector()
{

}

void VFHObjectDetector :: setObjectDatabase(const ObjectDatabasePtr& db)
{
    super::setObjectDatabase(db);

    if (!db) return;

    m_vfh_indexer = new VFHFeatureIndexer(objectDatabase(), LocatedFeature::Feature_VFH);
    TimeCount pc ("loading sift indexer");
    m_vfh_indexer->loadOrBuild();
    pc.stop();
}

void VFHObjectDetector::findObjects()
{
    ntk_dbg(1) << "Detecting objects!";
    m_matches.clear();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    rgbdImageToPointCloud(*cloud, analyzedImage());

    TableObjectDetector<pcl::PointXYZ> table_detector;
    table_detector.setDepthLimits(-2, -0.5);
    table_detector.setObjectVoxelSize(0.003); // 3 mm voxels.
    table_detector.setObjectHeightLimits(0.02, 0.5);
    table_detector.setMaxDistToPlane(0.1);
    table_detector.detect(cloud);
    int cluster_index = table_detector.getMostCentralCluster();
    if (cluster_index < 0)
    {
        ntk_dbg(1) << "No valid cluster found.";
        return;
    }

    TableObjectRGBDModeler modeler;
    modeler.feedFromTableObjectDetector(table_detector, cluster_index);
    modeler.addNewView(analyzedImage(), *analyzedImage().calibration()->depth_pose);
    modeler.computeSurfaceMesh();

    const std::vector<cv::Point3f>& object_points = modeler.currentMesh().vertices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    vectorToPointCloud<pcl::PointXYZ>(*object_cloud, object_points);

    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
    LocatedFeature feature;
    compute_vfh_features(feature, vfhs, object_cloud);
    ntk_dbg_print(vfhs->points.size(), 1);
    pcl::io::savePCDFileASCII("debug_object.pcd", *object_cloud);
    pcl::io::savePCDFileASCII("debug_signatures.pcd", *vfhs);

    VFHFeatureIndexer::MatchResults matches = m_vfh_indexer->findMatches(feature);
    if (matches.matches.size() < 1)
    {
        ntk_dbg(1) << "No matches found.";
        return;
    }

    ntk_dbg(1) << "=== BEST OBJECT ===";
    ntk_dbg_print(matches.matches[0].point->visualObjectView().name(), 1);
    ntk_dbg_print(matches.matches[0].score, 1);

    const VisualObjectView& model = matches.matches[0].point->visualObjectView();
    ObjectPosePtr pose (new ObjectPose(m_data, model, AffineTransform()));
    VFHObjectMatch match (m_data, pose);
    match.setScore(-matches.matches[0].score);
    match.setMatchedPoints(object_points);
    m_matches.push_back(match);
}

} // ntk

namespace ntk
{

VFHFeatureIndexer :: VFHFeatureIndexer(const ObjectDatabase& db, LocatedFeature::FeatureType type)
    : FeatureIndexer(db, type)
{

}

VFHFeatureIndexer :: ~VFHFeatureIndexer()
{
    foreach_idx(i, m_points)
            delete m_points[i];
}

unsigned VFHFeatureIndexer :: nbPointsInObjectView(const VisualObjectView& obj) const
{
    ntk_assert(obj.id().view_id_in_database < m_objects_view_data.size(), "Outbound access.");
    return m_objects_view_data[obj.id().view_id_in_database].nb_vfh_points;
}

VFHFeatureIndexer::MatchResults VFHFeatureIndexer :: findMatches(const LocatedFeature& p) const
{
    float min_dist = FLT_MAX;
    const LocatedFeature* best_match = 0;
    foreach_idx(i, m_points)
    {
        const LocatedFeature* f = m_points[i];
        ntk_dbg_print(p.descriptor().hasFloatDesc(), 1);
        ntk_dbg_print(f->descriptor().hasFloatDesc(), 1);
        ntk_dbg_print(p.descriptor().size(), 1);
        ntk_dbg_print(f->descriptor().size(), 1);
        double d = euclidian_distance(p.descriptor(), f->descriptor(), min_dist);
        ntk_dbg(1) << "======";
        ntk_dbg_print(f->visualObjectView().name(), 1);
        ntk_dbg_print(d, 1);
        if (d < min_dist)
        {
            min_dist = d;
            best_match = f;
        }
    }

    MatchResults results;
    if (best_match)
    {
        Match match(best_match, min_dist);
        results.matches.push_back(match);
    }

    return results;
}

void VFHFeatureIndexer :: rebuild()
{
    m_points.clear();
    m_points_data.clear();
    m_objects_view_data.resize(m_db.nbVisualObjectsViews());
    for (unsigned i = 0; i < m_db.nbVisualObjects(); ++i)
    {
        const VisualObject& obj = m_db.visualObject(i);
        const VisualObjectView& view = obj.views()[0];
        ntk_dbg(1) << "Computing index for object " << obj.name();

        ntk::Mesh mesh;
        try {
            obj.loadMesh(mesh);
        }
        catch (const std::exception& e)
        {
            fatal_error(e.what());
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        vectorToPointCloud<pcl::PointXYZ>(*object_cloud, mesh.vertices);
        pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
        LocatedFeature* feature = new LocatedFeature();
        compute_vfh_features(*feature, vfhs, object_cloud);

        feature->setIdInIndexer(m_points.size());
        feature->setVisualObjectView(view);
        m_points.push_back(feature);

        PointData point_data; point_data.visual_object_id = view.id();
        m_points_data.push_back(point_data);

        ViewData d; d.nb_vfh_points = 1;
        m_objects_view_data[view.id().view_id_in_database] = d;
    }
}

void VFHFeatureIndexer :: fillXmlElement(XMLNode& element) const
{
    setXmlAttribute(element, "database_id", m_db.uniqueId());
    int nb_objects = objectDatabase().nbVisualObjectsViews();
    setXmlAttribute(element, "nb_objects", nb_objects);

    setXmlAttribute(element, "nb_vfh_points", nbPoints());

    foreach_idx(i, m_objects_view_data)
    {
        XMLNode child = addXmlChildAtBeginning(element, "object_data", m_objects_view_data[i]);
        setXmlAttribute(child, "id", (int)i);
    }

    foreach_idx(i, m_points)
    {
        XMLNode child = addXmlChildAtBeginning(element, "vfh_point_data", m_points_data[i]);
        setXmlAttribute(child, "id", m_points[i]->idInIndexer());
        addXmlChild(child, "raw_data", *m_points[i]);
    }
}

void VFHFeatureIndexer :: loadFromXmlElement(const XMLNode& element)
{
    int database_id;
    loadFromXmlAttribute(element, "database_id", database_id);
    if (database_id != objectDatabase().uniqueId())
        ntk_throw_exception("Index does not correspond to database id.");

    int nb_objects; loadFromXmlAttribute(element, "nb_objects", nb_objects);
    if (nb_objects != (int)objectDatabase().nbVisualObjectsViews())
        ntk_throw_exception("Sift index does not have the same number of objects than database");

    int nb_vfh_points; loadFromXmlAttribute(element, "nb_vfh_points", nb_vfh_points);
    if (nb_vfh_points < 0)
        ntk_throw_exception("nb_vfh_points cannot be negative");

    m_objects_view_data.clear(); m_objects_view_data.resize(nb_objects);
    m_points_data.clear(); m_points_data.resize(nb_vfh_points);
    m_points.clear(); m_points.resize(nb_vfh_points, 0);

    for (int i = 0; i < element.nChildNode(); ++i)
    {
        XMLNode e = element.getChildNode(i);

        if (e.getNameAsString() == "object_data")
        {
            int id; loadFromXmlAttribute(e, "id", id);
            ntk_assert(m_objects_view_data[id].nb_vfh_points == -1, "Multiple object data");
            m_objects_view_data[id].loadFromXmlElement(e);
        }

        if (e.getNameAsString() == "vfh_point_data")
        {
            LocatedFeature* point = new LocatedFeature();
            int id_in_indexer; loadFromXmlAttribute(e, "id", id_in_indexer);
            ntk_assert(id_in_indexer >= 0 && id_in_indexer < nb_vfh_points, "Bad point id.");
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
            ntk_assert(m_objects_view_data[i].nb_vfh_points >= 0, "Object data missing.");

    foreach_idx(i, m_points_data)
            ntk_assert(m_points_data[i].visual_object_id.isValid(), "Point data missing.");

    foreach_idx(i, m_points)
            ntk_assert(m_points[i] != 0, "Sift point missing.");
}

} // ntk
