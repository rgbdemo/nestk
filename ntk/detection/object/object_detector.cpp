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

#include "object_detector.h"
#include "object_database.h"
#include "visual_object.h"

using namespace ntk;
using namespace cv;

namespace ntk
{

  ObjectDetector :: ObjectDetector() : m_is_running(false), m_no_multiple_instances(true)
  {}

  ObjectDetector :: ~ObjectDetector()
  {}

  void ObjectDetector :: setAnalyzedImage(const RGBDImage& image)
  {
    image.copyTo(m_data.image);
  }
  
} // end of avs
