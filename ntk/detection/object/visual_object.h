//
// visual_object.h
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

#ifndef FROL_visual_object_H
# define FROL_visual_object_H

# include <ntk/ntk.h>
# include <ntk/mesh/mesh.h>

# include "visual_object_view.h"

namespace ntk
{

  class ObjectDatabase;

  class VisualObject : public ntk::XmlSerializable, public ntk::XmlContext
  {
    static const std::string xml_filename;

  public:
    VisualObject() : m_database(0), m_id_in_db(-1), m_enabled(true),
                     m_z_correction_a(1), m_z_correction_b(0)
    {}
    VisualObject(const ObjectDatabase* database, 
                 int id_in_db,
                 const std::string& name);

  public:
    virtual void fillXmlElement(XMLNode& element) const;
    virtual void loadFromXmlElement(const XMLNode& element);

  public:
    bool tryLoad();
    void loadOrBuild();

  public:
    void disable() { m_enabled = false; }
    void enable() { m_enabled = true; }
    bool isEnabled() const { return m_enabled; }

  public:
    int idInDatabase() const { return m_id_in_db; }
    const ObjectDatabase* database() const { return m_database; }
    void setDatabase(const ObjectDatabase& db, int id_in_db)
    { m_database = &db; m_id_in_db = id_in_db; }

    const std::string& name() const { return m_name; }
    const std::string& directory() const { return m_dir; }
    const std::vector<VisualObjectView>& views() const { return m_views; }
    std::string plyFile() const { return m_dir + "/avs_metadata/corrected_mesh.ply"; }
    void loadMesh(ntk::Mesh& mesh) const;
    void loadMeshWithCache(ntk::Mesh& mesh) const;

    const ntk::Rect3f& boundingBox() const { return m_bounding_box; }
    bool has3DModel() const { return !boundingBox().isEmpty(); }

    // input is Z_Model * S_Image / S_Model
    // output is ideal Z_Image in metros
    double getCorrectedIdealZ (double z_model_times_scale_ratio) const
    { return z_model_times_scale_ratio * m_z_correction_a + m_z_correction_b; }

    bool hasZCorrection() const { return !ntk::flt_eq(m_z_correction_a, 0, 1e-5); }

  private:
    const ObjectDatabase* m_database;
    ntk::Rect3f m_bounding_box;
    int m_id_in_db;
    std::string m_name;
    std::string m_dir;
    std::vector<VisualObjectView> m_views;
    bool m_enabled;
    int m_unique_id;
    double m_z_correction_a;
    double m_z_correction_b;
    mutable ntk::Mesh m_cached_mesh;
  };
  ntk_ptr_typedefs(VisualObject);

} // end of avs

#endif // ndef FROL_visual_object_H
