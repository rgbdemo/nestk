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


#include "calibration.h"

#include <ntk/ntk.h>
#include <ntk/utils/opencv_utils.h>
#include <ntk/utils/stl.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/numeric/utils.h>
#include <ntk/mesh/mesh.h>

#include <opencv2/highgui/highgui.hpp>

#include <QDir>

using namespace ntk;
using namespace cv;

class PlaneEstimator;
class ModelAcquisitionWindow;
class ModelAcquisitionController;
class ObjectDetector;

namespace ntk
{

void calibrationCorners(const std::string& image_name,
                        const std::string& window_name,
                        int pattern_width, int pattern_height,
                        std::vector<Point2f>& corners,
                        const cv::Mat& image,
                        float scale_factor,
                        PatternType pattern,
                        cv::Mat3b* debug_image)
{
    Size pattern_size (pattern_width, pattern_height);

    cv::Mat scaled_image;

    if (ntk::flt_eq(scale_factor, 1))
    {
        scaled_image = image.clone();
    }
    else
    {
        cv::resize(image, scaled_image,
                   Size(image.cols*scale_factor, image.rows*scale_factor),
                   scale_factor, scale_factor, cv::INTER_CUBIC);
    }

    int flags = CV_CALIB_CB_NORMALIZE_IMAGE|CV_CALIB_CB_ADAPTIVE_THRESH;
    bool ok = true;
    switch (pattern)
    {
    case PatternChessboard: {
        ok = findChessboardCorners(scaled_image,
                                   pattern_size,
                                   corners,
                                   flags);

        if (!ok)
        {
            flags = CV_CALIB_CB_NORMALIZE_IMAGE;
            ok = findChessboardCorners(scaled_image,
                                       pattern_size,
                                       corners,
                                       flags);
        }

        if (!ok)
        {
            flags = CV_CALIB_CB_ADAPTIVE_THRESH;
            ok = findChessboardCorners(scaled_image,
                                       pattern_size,
                                       corners,
                                       flags);
        }

        if (!ok)
        {
            flags = 0;
            ok = findChessboardCorners(scaled_image,
                                       pattern_size,
                                       corners,
                                       flags);
        }
        break;
    }
    case PatternCircles: {
#ifdef HAVE_OPENCV_GREATER_THAN_2_2
        ok = findCirclesGrid( scaled_image, pattern_size, corners );
#else
        ntk_throw_exception("Circles pattern is not supported with OpenCV < 2.3");
#endif
        break;
    }
    case PatternAsymCircles: {
#ifdef HAVE_OPENCV_GREATER_THAN_2_2
        ok = findCirclesGrid( scaled_image, pattern_size, corners, CALIB_CB_ASYMMETRIC_GRID);
#else
        ntk_throw_exception("Circles pattern is not supported with OpenCV < 2.3");
#endif
        break;
    }
    }
    cv::Mat draw_image = scaled_image;

    cv::Mat gray_image;
    cvtColor(image, gray_image, CV_BGR2GRAY);
    if (ok && pattern == PatternChessboard)
    {
        cornerSubPix(gray_image, corners, Size(5,5), Size(-1,-1),
                     cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
    }

    if (ok && (!image_name.empty() || debug_image))
    {
        cv::Mat corner_matrix(corners.size(), 1, CV_32FC2);
        for (int row = 0; row < corners.size(); ++row)
            corner_matrix.at<Point2f>(row,0) = corners[row];

        drawChessboardCorners(draw_image, pattern_size, corner_matrix, ok);

        if (debug_image)
        {
            draw_image.copyTo(*debug_image);
        }

        if (!image_name.empty())
        {
            ntk_dbg_print(image_name, 1);
            imwrite(image_name + ".corners.png", draw_image);
        }

        if (!window_name.empty())
        {
            imshow(window_name, draw_image);
            waitKey(10);
        }
    }

    if (ok)
    {
        for (int i = 0; i < corners.size(); ++i)
        {
            corners[i].x /= scale_factor;
            corners[i].y /= scale_factor;
        }
    }

    if (!ok)
    {
        corners.clear();
        return;
    }
}

void crop_image(cv::Mat& image, cv::Size s)
{
    cv::Mat roi = image(cv::Rect((image.cols-s.width)/2.0,
                                 (image.rows-s.height)/2.0,
                                 s.width,
                                 s.height));
    cv::Mat new_image = cv::Mat(s, image.type());
    roi.copyTo(new_image);
    image = new_image;
}

void depth_distortionance_to_depth(cv::Mat1f& depth_im,
                                   const RGBDCalibration& calib,
                                   const cv::Mat1f& amplitude)
{
    double cx = calib.depth_intrinsics(0,2);
    double cy = calib.depth_intrinsics(1,2);
    double fx = calib.depth_intrinsics(0,0);
    double fy = calib.depth_intrinsics(1,1);

    for_all_rc(depth_im)
    {
        double orig_depth = depth_im(r,c);

        double dx = c-cx;
        double dy = r-cy;

        Point3f v(dx/fx, dy/fy, 1);
        double norm = sqrt(v.dot(v));
        v = v * (1.0/norm);
        v *= orig_depth;

        double depth_z = v.z;
        depth_im(r,c) = depth_z;
    }
}

void calibrationPattern(std::vector< std::vector<Point3f> >& output,
                        int pattern_width,
                        int pattern_height,
                        float square_size,
                        int nb_images)
{
    const int nb_corners = pattern_width * pattern_height;

    output.resize(nb_images);
    for(int i = 0; i < nb_images; ++i)
    {
        output[i].resize(nb_corners);
        for(int j = 0; j < pattern_height; ++j)
            for(int k = 0; k < pattern_width; ++k)
            {
                output[i][j*pattern_width+k] = Point3f(k*square_size, j*square_size, 0);
            }
    }
}

void calibrationPattern(std::vector<cv::Point3f> & output,
                        int pattern_width,
                        int pattern_height,
                        float square_size)
{
    const int nb_corners = pattern_width * pattern_height;


    output.resize(nb_corners);
    for(int j = 0; j < pattern_height; ++j)
        for(int k = 0; k < pattern_width; ++k)
        {
            output[j*pattern_width+k] = Point3f(k*square_size, j*square_size, 0);
        }

}

void estimate_checkerboard_pose(const std::vector<Point3f>& model,
                                const std::vector<Point2f>& img_points,
                                const cv::Mat1d& calib_matrix,
                                cv::Mat1f& H)
{
    cv::Mat1f to_open_cv (4,4);
    setIdentity(to_open_cv);
    to_open_cv(1,1) = -1;
    to_open_cv(2,2) = -1;
    cv::Mat1f from_open_cv = to_open_cv.inv();

    Mat3f model_mat(model.size(), 1); CvMat c_model_mat = model_mat;
    for_all_rc(model_mat) model_mat(r, 0) = Vec3f(model[r].x, -model[r].y, -model[r].z);

    // First image, for model pose.

    Mat2f point_mat(img_points.size(), 1); CvMat c_point_mat = point_mat;
    for_all_rc(point_mat) point_mat(r, 0) = Vec2f(img_points[r].x, img_points[r].y);

    Mat1f rvec (3,1); rvec = 0; CvMat c_rvec = rvec;
    Mat1f tvec (3,1); tvec = 0; CvMat c_tvec = tvec;

    CvMat c_calib_mat = calib_matrix;
    cvFindExtrinsicCameraParams2(&c_model_mat,
                                 &c_point_mat,
                                 &c_calib_mat,
                                 0, &c_rvec, &c_tvec);

    cv::Mat1f rot(3,3); CvMat c_rot = rot;
    cvRodrigues2(&c_rvec, &c_rot);

    H = cv::Mat1f(4,4);
    setIdentity(H);
    cv::Mat1f H_rot = H(Rect(0,0,3,3));
    rot.copyTo(H_rot);
    H(0,3) = tvec(0,0);
    H(1,3) = tvec(1,0);
    H(2,3) = tvec(2,0);
    ntk_dbg_print(H, 1);

    H = from_open_cv * H * to_open_cv;
}

void showCheckerboardCorners(const cv::Mat3b& image, const std::vector<Point2f>& corners, int wait_time)
{
    cv::Mat3b debug_img;
    image.copyTo(debug_img);
    foreach_idx(i, corners)
    {
        Point2i p = corners[i];
        Rect r (p+Point2i(-2,-2),cv::Size(4,4));
        cv::rectangle(debug_img, r, Scalar(255,0,0,255));
        cv::circle(debug_img, p, 8, Scalar(0,0,255,255));
    }
    imshow("corners", debug_img);
    cv::waitKey(wait_time);
}

double computeCalibrationError(const cv::Mat& F,
                               const std::vector<std::vector<Point2f> >& rgb_corners,
                               const std::vector<std::vector<Point2f> >& depth_corners)
{
    std::vector<cv::Point2f> points_in_rgb;
    for (int i = 0; i < rgb_corners.size(); ++i)
        for (int j = 0; j < rgb_corners[i].size(); ++j)
            points_in_rgb.push_back(rgb_corners[i][j]);

    std::vector<cv::Point2f> points_in_depth;
    for (int i = 0; i < depth_corners.size(); ++i)
        for (int j = 0; j < depth_corners[i].size(); ++j)
            points_in_depth.push_back(depth_corners[i][j]);

    std::vector<Vec3f> lines_in_depth;
    std::vector<Vec3f> lines_in_rgb;

    cv::computeCorrespondEpilines(cv::Mat(points_in_rgb), 1, F, lines_in_depth);
    cv::computeCorrespondEpilines(cv::Mat(points_in_depth), 2, F, lines_in_rgb);

    double avgErr = 0;
    for(int i = 0; i < points_in_rgb.size(); ++i)
    {
        double err = fabs(points_in_rgb[i].x*lines_in_rgb[i][0] +
                          points_in_rgb[i].y*lines_in_rgb[i][1] + lines_in_rgb[i][2]);
        avgErr += err;
    }

    for(int i = 0; i < points_in_depth.size(); ++i)
    {
        double err = fabs(points_in_depth[i].x*lines_in_depth[i][0] +
                          points_in_depth[i].y*lines_in_depth[i][1] + lines_in_depth[i][2]);
        avgErr += err;
    }

    return avgErr / (points_in_rgb.size() + points_in_depth.size());
}

void kinect_shift_ir_to_depth(cv::Mat3b& im, bool highres)
{
    imwrite("/tmp/before.png", im);
    float factor = highres ? 2.0f : 1.0f;
    cv::Mat1f t (2, 3);
    t = 0.f;
    t(0,0) = 1;
    t(1,1) = 1;
    t(0,2) = -4.8f * factor;
    t(1,2) = -3.9f * factor;
    cv::Mat3b tmp;
    warpAffine(im, tmp, t, im.size());
    im = tmp;
    imwrite("/tmp/after.png", im);
}

void loadImageList(const QStringList& view_dirs,
                   RGBDProcessor *processor,
                   RGBDCalibrationPtr calibration,
                   std::vector<RGBDImage>& images)
{
    images.clear();
    for (int i_view_dir = 0; i_view_dir < view_dirs.size(); ++i_view_dir)
    {
        RGBDImage image;
        image.loadFromDir(QDir(view_dirs[i_view_dir]).absolutePath().toStdString(), calibration, processor);
        images.push_back(image);
    }
}

void loadImageList(const QDir& image_dir,
                   const QStringList& view_list,
                   ntk::RGBDProcessor* processor,
                   RGBDCalibrationPtr calibration,
                   std::vector<RGBDImage>& images)
{
    QStringList view_dirs;
    for (int i_image = 0; i_image < view_list.size(); ++i_image)
    {
        QString filename = view_list[i_image];
        view_dirs.append(image_dir.absoluteFilePath(filename));
    }

    loadImageList(view_dirs, processor, calibration, images);
}

void getCalibratedCheckerboardCorners(const std::vector<RGBDImage>& images,
                                      int pattern_width,
                                      int pattern_height,
                                      PatternType pattern_type,
                                      std::vector< std::vector<Point2f> >& all_corners,
                                      std::vector< std::vector<Point2f> >& good_corners,
                                      bool show_corners,
                                      bool use_intensity)
{
    static int k = 0;
    k++;

    good_corners.clear();
    all_corners.resize(images.size());
    for (int i_image = 0; i_image < images.size(); ++i_image)
    {
        const RGBDImage& image = images[i_image];

        cv::Mat3b color_image;
        if (use_intensity)
        {
            color_image = toMat3b(normalize_toMat1b(image.intensity()));
            // kinect_shift_ir_to_depth(color_image, true);
            // FIXME: quite tricky but works for some cases.
#if 0
            for_all_rc(color_image)
            {
                int v = color_image(r,c)[0];
                if (v > 50)
                    color_image(r,c) = cv::Vec3b(255,255,255);
                else
                {
                    v = v*255/50;
                    color_image(r,c) = cv::Vec3b(v, v, v);
                }
            }
#endif
        }
        else
            color_image = image.rgb();
        imwrite("debug_color.png", color_image);

        std::vector<Point2f> current_view_corners;
        calibrationCorners(cv::format("corners%02d-%02d", k, i_image), "",
                           pattern_width, pattern_height,
                           current_view_corners, color_image, 1,
                           pattern_type);

        if (current_view_corners.size() == pattern_height*pattern_width)
        {
            all_corners[i_image] = current_view_corners;
            good_corners.push_back(current_view_corners);
            if (show_corners)
                showCheckerboardCorners(color_image, current_view_corners, 1);
        }
        else
        {
            ntk_dbg(0) << "Warning: corners not detected";
            all_corners[i_image].resize(0);
        }
    }
}

void calibrateStereoFromCheckerboard(const std::vector< std::vector<Point2f> >& undistorted_ref_corners,
                                     const std::vector< std::vector<Point2f> >& undistorted_corners,
                                     int pattern_width,
                                     int pattern_height,
                                     float pattern_size,
                                     ntk::RGBDCalibration& calibration,
                                     bool use_intensity)
{
    ntk_assert(undistorted_ref_corners.size() == undistorted_corners.size(), "Size should be equal.");
    std::vector< std::vector<Point2f> > undistorted_good_corners;
    std::vector< std::vector<Point2f> > undistorted_good_ref_corners;

    foreach_idx(i, undistorted_ref_corners)
    {
        if (undistorted_ref_corners[i].size() > 0 && undistorted_corners[i].size() > 0)
        {
            ntk_assert(undistorted_ref_corners[i].size() == undistorted_corners[i].size(),
                       "Sizes should be equal.");
            undistorted_good_ref_corners.push_back(undistorted_ref_corners[i]);
            undistorted_good_corners.push_back(undistorted_corners[i]);
        }
    }

    std::vector< std::vector<Point3f> > pattern_points;
    calibrationPattern(pattern_points,
                       pattern_width,  pattern_height, pattern_size,
                       undistorted_good_ref_corners.size());

    cv::Mat R, T;
    cv::Mat E(3,3,CV_64F),F(3,3,CV_64F);
    cv::Mat zero_dist (calibration.depth_distortion.size(), calibration.depth_distortion.type());
    zero_dist = Scalar(0);

    cv::Size image_size;
    if (use_intensity)
    {
        image_size = calibration.infraredSize();
    }
    else
    {
        image_size = calibration.rgbSize();
    }

    cv::Mat1d intrinsics;
    if (use_intensity)
    {
        intrinsics = calibration.infrared_intrinsics;
    }
    else
    {
        intrinsics = calibration.rgb_intrinsics;
    }

    stereoCalibrate(pattern_points,
                    undistorted_good_ref_corners, undistorted_good_corners,                    
                    intrinsics, zero_dist, intrinsics, zero_dist,
                    image_size,
                    R, T, E, F,
                    TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 50, 1e-6),
                    CALIB_FIX_INTRINSIC|CALIB_SAME_FOCAL_LENGTH);

    // OpenCV coords has y down and z toward scene.
    // OpenGL classical 3d coords has y up and z backwards
    // This is the transform matrix.

    cv::Mat1d to_gl_base(3,3); setIdentity(to_gl_base);
    to_gl_base(1,1) = -1;
    to_gl_base(2,2) = -1;

    cv::Mat1d new_R = to_gl_base.inv() * R * to_gl_base;
    cv::Mat1d new_T = to_gl_base * (T);

    new_R.copyTo(R);
    new_T.copyTo(T);

    double error = computeCalibrationError(F, undistorted_good_ref_corners, undistorted_good_corners);
    std::cout << "Average pixel reprojection error: " << error << std::endl;

    R.copyTo(calibration.R_extrinsics);
    T.copyTo(calibration.T_extrinsics);
}

void calibrate_kinect_depth_infrared(const std::vector<RGBDImage>& images,
                                     const std::vector< std::vector<Point2f> >& good_corners,
                                     RGBDCalibration& calibration,
                                     int pattern_width,
                                     int pattern_height,
                                     float pattern_size,
                                     ntk::PatternType pattern_type,
                                     bool ignore_distortions,
                                     bool fix_center,
                                     int default_flags)
{
    std::vector< std::vector<Point3f> > pattern_points;
    calibrationPattern(pattern_points,
                       pattern_width, pattern_height, pattern_size,
                       good_corners.size());

    ntk_assert(pattern_points.size() == good_corners.size(), "Invalid points size");

    int flags = default_flags;
    if (ignore_distortions)
        flags |= CV_CALIB_ZERO_TANGENT_DIST;
    if (fix_center)
        flags |= CV_CALIB_FIX_PRINCIPAL_POINT;

    std::vector<Mat> rvecs, tvecs;
    double reprojection_error = calibrateCamera(pattern_points, good_corners, calibration.infraredSize(),
                                                calibration.infrared_intrinsics, calibration.infrared_distortion,
                                                rvecs, tvecs, flags);
    ntk_dbg_print(reprojection_error, 0);

    if (ignore_distortions)
        calibration.infrared_distortion = 0.f;
}

void calibrate_kinect_rgb(const std::vector<RGBDImage>& images,
                          const std::vector< std::vector<Point2f> >& good_corners,
                          RGBDCalibration& calibration,
                          int pattern_width,
                          int pattern_height,
                          float pattern_size,
                          ntk::PatternType pattern_type,
                          bool ignore_distortions,
                          bool fix_center,
                          int default_flags)
{
    std::vector< std::vector<Point3f> > pattern_points;
    calibrationPattern(pattern_points,
                       pattern_width, pattern_height, pattern_size,
                       good_corners.size());

    ntk_assert(pattern_points.size() == good_corners.size(), "Invalid points size");

    int flags = default_flags;
    if (ignore_distortions)
        flags |= CV_CALIB_ZERO_TANGENT_DIST;
    if (fix_center)
        flags |= CV_CALIB_FIX_PRINCIPAL_POINT;

    std::vector<Mat> rvecs, tvecs;
    double reprojection_error = calibrateCamera(pattern_points, good_corners, calibration.rawRgbSize(),
                                                calibration.rgb_intrinsics, calibration.rgb_distortion,
                                                rvecs, tvecs, flags);
    ntk_dbg_print(reprojection_error, 0);

    if (ignore_distortions)
        calibration.rgb_distortion = 0.f;
}

static float computeScaleFactorMean(const std::vector<Point2f>& current_view_corners,
                                    const RGBDImage& image,
                                    int pattern_width, int pattern_height, float pattern_size)
{
    std::vector< std::vector<Point3f> > pattern_points;
    calibrationPattern(pattern_points,
                       pattern_width,  pattern_height, pattern_size,
                       1);

    float mean = 0;
    int n_values = 0;

    for (int i = 0; i < current_view_corners.size(); ++i)
    {
        float depth_i = image.mappedDepth()(current_view_corners[i].y, current_view_corners[i].x);
        ntk_dbg_print(depth_i, 1);
        if (depth_i < 1e-5) continue;

        cv::Point3f p3d_i = image.calibration()->rgb_pose->unprojectFromImage(current_view_corners[i], depth_i);

        for (int j = (i+1); j < current_view_corners.size(); ++j)
        {
            float depth_j = image.mappedDepth()(current_view_corners[j].y, current_view_corners[j].x);
            if (depth_j < 1e-5) continue;

            ntk_dbg_print(depth_j, 1);

            cv::Point3f p3d_j = image.calibration()->rgb_pose->unprojectFromImage(current_view_corners[j], depth_j);
            ntk_dbg_print(p3d_i, 1);
            ntk_dbg_print(p3d_j, 1);

            float kinect_size = cv::norm(p3d_i - p3d_j);
            ntk_dbg_print(kinect_size, 1);

            float real_size = cv::norm(pattern_points[0][i] - pattern_points[0][j]);
            ntk_dbg_print(real_size, 1);

            mean += (real_size / kinect_size);
            ++n_values;
        }
    }
    if (n_values > 0)
        mean /= n_values;
    else
        mean = 0.f;
    ntk_dbg_print(mean, 1);
    return mean;
}

float calibrate_kinect_scale_factor(const std::vector<RGBDImage>& images,
                                    const std::vector< std::vector<Point2f> >& corners,
                                    int pattern_width, int pattern_height, float pattern_size)
{
    float scale_factor_mean = 0;
    int n_factor_terms = 0;

    for (int i = 0; i < images.size(); ++i)
    {
        if (corners[i].size() == 0)
            continue;

        float scale = computeScaleFactorMean(corners[i], images[i], pattern_width, pattern_height, pattern_size);
        if (scale > 1e-5)
        {
            scale_factor_mean += scale;
            ++n_factor_terms;
        }
    }

    if (n_factor_terms > 0)
    {
        scale_factor_mean /= n_factor_terms;
        return scale_factor_mean;
    }

    return 0.f;
}

} // ntk
