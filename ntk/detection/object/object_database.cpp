//
// database.cc
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

#include "object_database.h"
#include "visual_object.h"

#include <ntk/ntk.h>

#include <QDir>

using namespace ntk;

namespace ntk
{
  const std::string ObjectDatabase::xml_filename = "avs_metadata/database.xml";

  ObjectDatabase :: ObjectDatabase(const std::string& dir)
    : m_dir(dir), m_id(-1)
  {
    loadOrBuild();
  }

  ObjectDatabase :: ~ObjectDatabase()
  {
    clearVisualObjects();
  }

  void ObjectDatabase::clearVisualObjects()
  {
    foreach_it(it, m_objects, std::vector<VisualObject*>)
      delete *it;
    m_objects.clear();
  }

  void ObjectDatabase :: fillXmlElement(XMLNode& element) const
  {
    setXmlAttribute(element, "id", m_id);
    foreach_const_it(it, m_objects, std::vector<VisualObject*>)
    {
      XMLNode child = element.addChild("object");
      setXmlAttribute(child, "name", (*it)->name());
    }
  }
  
  void ObjectDatabase:: loadFromXmlElement(const XMLNode& element)
  {  
    clearVisualObjects();
    loadFromXmlAttribute(element, "id", m_id);

    for (int i = 0; i < element.nChildNode(); ++i)
    {
      XMLNode e = element.getChildNode(i);
      if (e.getNameAsString() != "object") continue;

      const int object_id = m_objects.size();
      QString name;
      loadFromXmlAttribute(e, "name", name);
      VisualObject* obj = new VisualObject(this, object_id, name.toStdString());
      if (!obj->tryLoad())
        ntk_throw_exception("Could not load object!");
      m_objects.push_back(obj);
    }
  }  

  const VisualObject*
  ObjectDatabase :: visualObject(const std::string& name) const
  {
    foreach_const_it(it, m_objects, std::vector<VisualObject*>)
      if ((*it)->name() == name)
        return *it;
    return 0;
  }

  VisualObjectViewId
  ObjectDatabase :: viewIndexToFullViewId(int index) const
  {
    VisualObjectViewId id;
    id.view_id_in_database = index;
    id.object_id = 0;
    id.view_id_in_object = 0;

    int n_views = 0;
    for (int i = 0; n_views <= index && i < m_objects.size(); ++i)
    {
      if ((n_views + m_objects[i]->views().size()) > index)
      {
        id.object_id = i;
        id.view_id_in_object = index - n_views;
        break;
      }
      n_views += m_objects[i]->views().size();
    }
    return id;
  }

  const VisualObjectView*
  ObjectDatabase :: visualObjectView(const std::string& name) const
  {
    foreach_const_it(it, m_objects, std::vector<VisualObject*>)
    {
      foreach_idx(i, (*it)->views())
      {
        const VisualObjectView& view = (*it)->views()[i];
        if (view.name() == name)
          return &view;
      }
    }
    return 0;
  }

  bool ObjectDatabase :: tryLoad(const QFileInfoList& models)
  {
    QFileInfo xml_file ((m_dir + "/" + xml_filename).c_str());
    ntk_dbg(1) << "Trying to load: " << xml_file.absoluteFilePath();

    if (!xml_file.isFile())
    {
      ntk_dbg(1) << xml_file.absoluteFilePath() << " does not exist.";
      return false;
    }

    try
    {
      loadFromXml(xml_file, "database");
    }
    catch (const std::exception& e)
    {
      // Could not load the file.
      ntk_dbg(1) << "Database xml file could not be parsed: " << e.what();
      return false;
    }

    ntk_dbg(2) << "Visual objects loaded from: "  << xml_file.absoluteFilePath();

    bool sane_load = true;
    sane_load &= (m_objects.size() == models.size());
    ntk_dbg_print(m_objects.size(), 1);

    foreach_idx(i, m_objects)
    {
      sane_load &= (models[i].fileName().toStdString() == m_objects[i]->name());
      if (!sane_load) break;
    }

    if (sane_load)
    {
      ntk_dbg(1) << "Ok, database safely loaded.";
      return true;
    }
    else
    {
      ntk_dbg(1) << "Database has changed, removing it.";
      clearVisualObjects();
      return false;
    }
    return false;
  }

  bool ObjectDatabase :: removePrecomputedFiles()
  {
    QDir dir (m_dir.c_str()); dir.rmpath("avs_metadata");
    return true;
  }
  
  unsigned ObjectDatabase :: nbVisualObjectsViews() const
  {
    unsigned sum = 0;
    foreach_idx(i, m_objects)
    {
      sum += m_objects[i]->views().size();
    }
    return sum;
  }

  void ObjectDatabase :: loadOrBuild()
  {
    QDir db_dir (m_dir.c_str());
    QFileInfoList models;
    models = db_dir.entryInfoList(QStringList("*.model"), QDir::Dirs, QDir::Name);

    if (tryLoad(models) == true) return;
    clearVisualObjects();

    //cv::Mat im = cv::imread("test.png");

    ntk_dbg(1) << "Re-building db from files.";
    db_dir.rmpath("avs_metadata"); // they are obsolete.
    db_dir.mkdir("avs_metadata");

    ntk::TimeCount tc_load_db("Load database", 1);
    foreach_idx(i, models)
    {
      VisualObject* obj = new VisualObject(this, i,
                                           models[i].fileName().toStdString());
      obj->loadOrBuild();
      m_objects.push_back(obj);
    }
    tc_load_db.elapsedMsecs("-- loading: ");

    m_id = cv::RNG()();
    m_id *= m_id;

    QFileInfo xml_file (db_dir.filePath(xml_filename.c_str()));
    ntk_dbg(2) << "Saving db in: " << xml_file.absoluteFilePath();
    saveAsXml(xml_file, "database");
    tc_load_db.elapsedMsecs("-- after save: ");
  }

} // end of avs
