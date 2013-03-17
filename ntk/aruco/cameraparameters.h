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

#ifndef _Aruco_CameraParameters_H
#define  _Aruco_CameraParameters_H
// #include <opencv2/core/core.hpp>
#include <ntk/core.h>
#include <string>
using namespace std;
namespace aruco
{
/**\brief Parameters of the camera
 */

class CameraParameters
{
public:

    // 3x3 matrix (fx 0 cx, 0 fy cy, 0 0 1)
    cv::Mat  CameraMatrix;
    //4x1 matrix (k1,k2,p1,p2)
    cv::Mat  Distorsion;
    //size of the image
    cv::Size CamSize;

    /**Empty constructor
     */
    CameraParameters() ;
    /**Creates the object from the info passed
     * @param cameraMatrix 3x3 matrix (fx 0 cx, 0 fy cy, 0 0 1)
     * @param distorsionCoeff 4x1 matrix (k1,k2,p1,p2)
     * @param size image size
     */
    CameraParameters(cv::Mat cameraMatrix,cv::Mat distorsionCoeff,cv::Size size);
    /**Copy constructor
     */
    CameraParameters(const CameraParameters &CI) ;

    /**Indicates whether this object is valid
     */
    bool isValid()const {
        return CameraMatrix.rows!=0 && CameraMatrix.cols!=0  && Distorsion.rows!=0 && Distorsion.cols!=0 && CamSize.width!=-1 && CamSize.height!=-1;
    }
    /**Assign operator
    */
    CameraParameters & operator=(const CameraParameters &CI);
    /**Reads the camera parameters from a file generated using saveToFile.
     */
    void readFromFile(string path);
    /**Saves this to a file
     */
    void saveToFile(string path);
    
    /**Reads from a YAML file generated with the opencv2.2 calibration utility
     */
    void readFromXMLFile(string filePath);
    
    /**Adjust the parameters to the size of the image indicated
     */
    void resize(cv::Size size);

    /**Returns the location of the camera in the reference system given by the rotation and translation vectors passed
     * NOT TESTED
    */
    static cv::Point3f getCameraLocation(cv::Mat Rvec,cv::Mat Tvec); 

};

}
#endif


