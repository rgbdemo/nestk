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

#ifndef FROL_object_finder_H
# define FROL_object_finder_H

# include <ntk/ntk.h>
# include <ntk/camera/calibration.h>
# include "object_match.h"
# include "object_database.h"

namespace ntk
{

  struct ObjectDetectorData
  {
    ntk::RGBDImage image;
  };

  class VisualObject; ntk_ptr_typedefs(VisualObject);
  class ObjectDatabase; ntk_ptr_typedefs(ObjectDatabase);

  class ObjectDetector : public ntk::XmlSerializable, public ntk::XmlContext
  {
    public:
      ObjectDetector();
      virtual ~ObjectDetector();

    public:
      const ObjectDatabase& objectDatabase() const { return *m_database; }
      const ObjectDatabasePtr& objectDatabasePtr() { return m_database; }
      virtual void setObjectDatabase(const ObjectDatabasePtr& db) { m_database = db; }
      
      const ntk::RGBDImage& analyzedImage() const { return m_data.image; }
      const cv::Mat1f& analyzedImageDepth() const { return m_data.image.mappedDepth(); }
      const cv::Mat1b& analyzedImageAsGray() const { return m_data.image.rgbAsGray(); }
      const cv::Mat3b& analyzedImageAsColor() const { return m_data.image.rgb(); }
      virtual void setAnalyzedImage(const ntk::RGBDImage& image);

      const ObjectDetectorData& data() const { return m_data; }

      void setAllowMultipleInstance(bool enable) { m_no_multiple_instances = !enable; }

    public:
      bool isRunning() const { return m_is_running; }

    public:
      virtual unsigned nbObjectMatches() const = 0;
      virtual const ObjectMatch& objectMatch(unsigned idx) const = 0;
      virtual ObjectMatch& objectMatch(unsigned idx) = 0;
      virtual void keepOnlyBestMatch() = 0;

    public:
      virtual void findObjects() = 0;
      virtual void initializeFindObjects() { m_is_running = true; }
      virtual void finalizeFindObjects() { m_is_running = false; }

    protected:
      ObjectDatabasePtr m_database;
      ObjectDetectorData m_data;
      bool m_is_running;
      bool m_no_multiple_instances;
  };
  ntk_ptr_typedefs(ObjectDetector);

} // end of avs

#endif // ndef FROL_object_finder_H
