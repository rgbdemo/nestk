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
