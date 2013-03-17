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

#ifndef _Aruco_BoardDetector_H
#define _Aruco_BoardDetector_H
// #include <opencv2/core/core.hpp>
#include "board.h"
#include "cameraparameters.h"
using namespace std;
using namespace cv;
namespace aruco
{

/**\brief This class detects AR boards
*/
class BoardDetector
{
public:

    /** Given the markers detected, determines if there is the board passed
    * @param detectedMarkers result provided by aruco::ArMarkerDetector
    * @param BConf the board you want to see if is present
    * @param Bdetected output information of the detected board
    * @param camMatrix camera matrix with intrinsics
    * @param distCoeff camera distorsion coeff
    * @param camMatrix intrinsic camera information.
    * @param distCoeff camera distorsion coefficient. If set Mat() if is assumed no camera distorion
    * @param markerSizeMeters size of the marker sides expressed in meters
    * @return value indicating  the  likelihood of having found the marker
    */
    float detect(const vector<Marker> &detectedMarkers,const  BoardConfiguration &BConf, Board &Bdetected, Mat camMatrix=Mat(),Mat distCoeff=Mat(), float markerSizeMeters=-1 )throw (cv::Exception);
    float detect(const vector<Marker> &detectedMarkers,const  BoardConfiguration &BConf, Board &Bdetected, CameraParameters cp, float markerSizeMeters=-1 )throw (cv::Exception);
private:
    void rotateXAxis(Mat &rotation);
};

}
#endif

