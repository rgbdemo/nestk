#ifndef FROL_common_database_H
# define FROL_common_database_H

# include <ntk/ntk.h>

# include "visual_object.h"

namespace ntk
{

  class ObjectDatabase : public ntk::XmlSerializable, public ntk::XmlContext
  {
    public:
      static const std::string xml_filename;

    public:
      ObjectDatabase(const std::string& directory);
      virtual ~ObjectDatabase();
      void clearVisualObjects();

    public:
      virtual void fillXmlElement(XMLNode& element) const;
      virtual void loadFromXmlElement(const XMLNode& element);

    public:
      const std::string& directory() const { return m_dir; }

      unsigned nbVisualObjects() const { return m_objects.size(); }
      unsigned nbVisualObjectsViews() const;

      const VisualObject& visualObject(int index) const
      { 
        ntk_assert((size_t)index < m_objects.size(), "Out of bounds.");
        return *m_objects[index];
      }

      VisualObject& visualObject(int index)
      { 
        ntk_assert((size_t)index < m_objects.size(), "Out of bounds.");
        return *m_objects[index];
      }

      VisualObjectViewId viewIndexToFullViewId(int index) const;

      const VisualObject* visualObject(const std::string& name) const;

      const VisualObjectView& visualObjectView(int index) const
      {
        VisualObjectViewId id = viewIndexToFullViewId(index);
        return m_objects[id.object_id]->views()[id.view_id_in_object];
      }

      const VisualObjectView& visualObjectView(VisualObjectViewId id) const
      { return m_objects[id.object_id]->views()[id.view_id_in_object]; }

      const VisualObjectView* visualObjectView(const std::string& name) const;

      int uniqueId() const { return m_id; }

    protected:
      void loadOrBuild();

    private:
      bool tryLoad(const QFileInfoList& models);
      bool removePrecomputedFiles();

    private:
      std::string m_dir;
      std::vector<VisualObject*> m_objects;
      int m_id;
  };
  ntk_ptr_typedefs(ObjectDatabase);

} // end of avs

#endif // ndef FROL_common_database_H
