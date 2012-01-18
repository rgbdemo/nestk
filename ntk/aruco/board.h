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

#ifndef _Aruco_board_h
#define _Aruco_board_h
// #include <opencv/cv.h>
#include <string>
#include <vector>
#include "marker.h"
using namespace std;
namespace aruco{
/**\brief This class defines a board with several markers.
* Board contains several markers so that they are more robustly detected
*/
 
class BoardConfiguration
{
public:
    cv::Mat _markersId;// grid of marker ids. Represent the matrix of (integers) markers in the board
    int _markerSizePix, _markerDistancePix;//size in pixels of the marker side and the distance between markers

  /**
    */
    BoardConfiguration();

    /**
    */
    BoardConfiguration(const BoardConfiguration  &T);

    /**Saves the board info to a file
    */
    void saveToFile(string sfile)throw (cv::Exception);
    /**Reads board info from a file
    */
    void readFromFile(string sfile)throw (cv::Exception);

};

/**
*/
class Board:public vector<Marker>
{
public:
    BoardConfiguration conf;
    //matrices of rotation and translation respect to the camera
    cv::Mat Rvec,Tvec;
    float markerSizeMeters;
    /**
    */
    Board()
    {
        Rvec.create(3,1,CV_32FC1);
        Tvec.create(3,1,CV_32FC1);
        for (int i=0;i<3;i++)
            Tvec.at<float>(i,0)=Rvec.at<float>(i,0)=-999999;
	markerSizeMeters=-1;
    }

    /**Given the extrinsic camera parameters returns the GL_MODELVIEW matrix for opengl.
    * Setting this matrix, the reference corrdinate system will be set in this board
     */
    void glGetModelViewMatrix(double modelview_matrix[16])throw(cv::Exception);

    /**Creates a printable image of the board
     */

    static cv::Mat createBoardImage( cv::Size  gridSize,int MarkerSize,int MarkerDistance,unsigned int FirstMarkerID, BoardConfiguration& TInfo  ) throw (cv::Exception);


};
}

#endif
