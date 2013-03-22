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

#include "object_database.h"
#include "visual_object.h"
#include <ntk/ntk.h>
#include <ntk/mesh/ply.h>

#include <QDir>

#include <fstream>

using namespace ntk;

namespace ntk
{
const std::string VisualObject::xml_filename = "avs_metadata/model.xml";

VisualObject :: VisualObject(const ObjectDatabase* database,
                             int id_in_db,
                             const std::string& name)
    : m_database(database), m_id_in_db(id_in_db), m_name(name), m_enabled(true), m_unique_id(-1),
      m_z_correction_a(0), m_z_correction_b(0)
{
    m_dir = database->directory() + "/" + name;
}

void VisualObject :: fillXmlElement(XMLNode& element) const
{
    setXmlAttribute(element, "id", m_unique_id);
    setXmlAttribute(element, "z_correction_a", m_z_correction_a);
    setXmlAttribute(element, "z_correction_b", m_z_correction_b);
    addXmlRawTextDataChild(element, "bounding_box", m_bounding_box);
    foreach_idx(i, m_views)
            addXmlChild(element, "view", m_views[i]);
}

void VisualObject :: loadFromXmlElement(const XMLNode& element)
{
    loadFromXmlAttribute(element, "id", m_unique_id);
    loadFromXmlRawTextDataChild(element, "bounding_box", m_bounding_box);
    loadFromXmlAttribute(element, "z_correction_a", m_z_correction_a);
    loadFromXmlAttribute(element, "z_correction_b", m_z_correction_b);

    for (int i = 0; i < element.nChildNode(); ++i)
    {
        XMLNode e = element.getChildNode(i);
        if (e.getNameAsString() != "view") continue;
        VisualObjectViewId id;
        id.object_id = m_id_in_db;
        id.view_id_in_object = m_views.size();
        id.view_id_in_database = m_database->nbVisualObjectsViews() + id.view_id_in_object;
        VisualObjectView view (this, id);
        view.loadFromXmlElement(e, this);
        m_views.push_back(view);
    }
}

bool VisualObject :: tryLoad()
{
    QFileInfo xml_file ((m_dir + "/" + xml_filename).c_str());
    ntk_dbg(2) << "Trying to load object metadata: " << xml_file.absoluteFilePath();

    if (!xml_file.exists())
    {
        ntk_dbg(1) << xml_file.absoluteFilePath() << " does not exist.";
        return false;
    }

    try
    {
        loadFromXml(xml_file, "object");
    }
    catch (const std::exception& e)
    {
        ntk_dbg(1) << "Could not load object metadata." << e.what();
        return false;
    }

    QFileInfoList views;
    views = QDir(m_dir.c_str()).entryInfoList(QStringList("view????"), QDir::Dirs, QDir::Name);

    if (views.size() != m_views.size())
    {
        ntk_dbg_print(views.size(), 1);
        ntk_dbg_print(m_views.size(), 1);
        ntk_dbg(1) << "Number of views has changed.";
        return false;
    }

    // TODO: check for a checksum or something.

    return true;
}

void VisualObject :: loadMesh(ntk::Mesh& mesh) const
{
    mesh.loadFromPlyFile(plyFile().c_str());
}

void VisualObject :: loadMeshWithCache(ntk::Mesh& mesh) const
{
    if (m_cached_mesh.vertices.size() < 1)
        loadMesh(m_cached_mesh);
    mesh = m_cached_mesh;
}

void VisualObject :: loadOrBuild()
{
    ntk::TimeCount tc_load_obj("Load object", 1);
    ntk_throw_exception_if (!QFileInfo(m_dir.c_str()).isDir(), "Cannot build from invalid directory.");

    if (tryLoad()) return;

    QDir(m_dir.c_str()).rmpath("avs_metadata");
    QDir(m_dir.c_str()).mkdir("avs_metadata");
    m_views.clear();

    m_unique_id = cv::RNG()();
    m_unique_id *= m_unique_id;

    QFileInfoList views;
    views = QDir(m_dir.c_str()).entryInfoList(QStringList("view????"), QDir::Dirs, QDir::Name);
    ntk_dbg_print(views.size(), 1);

    tc_load_obj.elapsedMsecs(" -- before load views: ");
    for (int i = 0; i < views.size(); ++i)
    {
        QFileInfo image_dir = views[i];
        ntk_dbg_print(image_dir.absoluteFilePath(), 1);
        VisualObjectViewId id;
        id.object_id = m_id_in_db;
        id.view_id_in_object = m_views.size();
        id.view_id_in_database = m_database->nbVisualObjectsViews() + id.view_id_in_object;
        VisualObjectView view(this, image_dir.absoluteFilePath().toStdString(), id);
        m_views.push_back(view);
    }
    tc_load_obj.elapsedMsecs(" -- after load views: ");

    std::string ply_file = m_dir + "/mesh.ply";
    if (!QFileInfo(ply_file.c_str()).exists())
    {
        m_bounding_box = ntk::Rect3f();
        ntk_assert(!has3DModel(), "Bbox should be empty");
    }
    else
    {
        Mesh mesh;
        mesh.loadFromPlyFile(ply_file.c_str());
        m_bounding_box = ntk::bounding_box(mesh.vertices);
        const bool apply_correction = false;

        if (apply_correction)
        {
            cv::Point3f centroid = m_bounding_box.centroid();

            double rx=0,ry=0,rz=0;
            QFileInfo rotation_file ((m_dir + "/rotation.txt").c_str());
            if (rotation_file.exists())
            {
                ntk_dbg(1) << "Rotation file exists, using it.";
                std::ifstream f (rotation_file.absoluteFilePath().toUtf8());
                f >> rx >> ry >> rz;
                rx = deg_to_rad(rx);
                ry = deg_to_rad(ry);
                rz = deg_to_rad(rz);
                f.close();
            }

            cv::Vec3f euler_angles (rx,ry,rz);
            Pose3D pose;
            pose.applyTransformBefore(-centroid, euler_angles);

            foreach_idx(i, mesh.vertices)
            {
                mesh.vertices[i] = pose.cameraTransform(mesh.vertices[i]);
            }


            m_bounding_box = ntk::bounding_box(mesh.vertices);

            pose.invert();
            foreach_idx(i, m_views)
            {
                m_views[i].transformPose(pose.cvTranslation(), pose.cvEulerRotation());
            }
        } // apply_correction

        mesh.saveToPlyFile(plyFile().c_str());

    }
    tc_load_obj.elapsedMsecs(" -- after mesh correction: ");

    foreach_idx(i, m_views)
    {
        m_views[i].buildCache();
    }

    tc_load_obj.elapsedMsecs(" -- after build cache: ");

    QFileInfo z_correction_file ((m_dir + "/z_correction.txt").c_str());
    if (z_correction_file.exists())
    {
        std::ifstream f (z_correction_file.absoluteFilePath().toUtf8());
        f >> m_z_correction_a >> m_z_correction_b;
        f.close();
    }

    QFileInfo xml_file ((m_dir + "/" + xml_filename).c_str());
    ntk_dbg(2) << "Saving db in: " << xml_file.absoluteFilePath();
    saveAsXml(xml_file, "object");
    tc_load_obj.elapsedMsecs(" -- after save: ");
}

} // end of avs
