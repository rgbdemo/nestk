
#include "calibration.h"

#include <ntk/ntk.h>
#include <ntk/utils/opencv_utils.h>
#include <ntk/utils/stl.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/numeric/utils.h>
#include <ntk/mesh/mesh.h>

#include <opencv2/highgui/highgui.hpp>

using namespace ntk;
using namespace cv;

namespace ntk
{

ProjectorCalibration :: ~ProjectorCalibration()
{
	if (pose)
		delete pose;
}

void ProjectorCalibration :: loadFromFile(const char* filename)
{
    QFileInfo f (filename);
    ntk_throw_exception_if(!f.exists(), "Could not find calibration file.");
    cv::FileStorage calibration_file (filename, CV_STORAGE_READ);
    readMatrix(calibration_file, "proj_intrinsics", intrinsics);
    readMatrix(calibration_file, "proj_distortion", distortion);
    readMatrix(calibration_file, "R", R);
    readMatrix(calibration_file, "T", T);
    cv::Mat1i size_mat;
    readMatrix(calibration_file, "proj_size", size_mat);
    proj_size = cv::Size(size_mat(0,0), size_mat(0,1));
    calibration_file.release();

    pose = new Pose3D();
    pose->toRightCamera(intrinsics, R, T);

    initUndistortRectifyMap(intrinsics, distortion,
                            Mat(), intrinsics, proj_size, CV_16SC2,
                            undistort_map1, undistort_map2);
}

} // ntk
