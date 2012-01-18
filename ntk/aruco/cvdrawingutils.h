/**
 * ArUco: Augmented Reality library from the University of Cordoba
 *
 * ArUco is a minimal C++ library for detection of Augmented Reality
 * markers based on OpenCv exclusively.
 *
 * It is an educational project to show student how to detect augmented
 * reality markers and it is provided under BSD license.
 *
 * This software is released under BSD license, see
 * http://www.uco.es/investiga/grupos/ava/node/26 for details.
 */

#ifndef _ArUco_DrawUtils_H_
#define _ArUco_DrawUtils_H_
#include "aruco.h"

namespace aruco
{
  /**\brief A set of functions to draw in opencv images
   */
  class CvDrawingUtils
  {
  public:
    
    static void draw3dAxis(cv::Mat &Image,Marker &m,CameraParameters &CP);
    
    static void draw3dCube(cv::Mat &Image,Marker &m,CameraParameters &CP);
    
    static void draw3dAxis(cv::Mat &Image,Board &m,CameraParameters &CP);
    
    static void draw3dCube(cv::Mat &Image,Board &m,CameraParameters &CP);

  };
};

#endif

