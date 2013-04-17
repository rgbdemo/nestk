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


#include "rgbd_processor.h"
#include <ntk/utils/opencv_utils.h>
#include <ntk/utils/time.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/image/bilateral_filter.h>
#include <ntk/camera/rgbd_calibration.h>

#ifdef NESTK_USE_PMDSDK
#include <ntk/camera/pmd_grabber.h>
#endif

#ifdef NESTK_USE_PCL
#include <ntk/mesh/pcl_utils.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#endif

#ifdef HAVE_OPENCV_GREATER_THAN_2_4_0
# include "opencv2/photo/photo.hpp"
#endif

// FIXME: The KdTreeFLANN class template instantiation (from kdtree_flann.h)
// indirectly triggered later in this compilation unit will fail if the cv
// namespace is pulled at the top-level one. Fully-qualify cv symbols, for now.
// using namespace cv;

namespace ntk
{

void subsampleDepth (const cv::Mat1f& depth_im, cv::Mat1f& subsampled_im, int factor)
{
    subsampled_im.create (depth_im.rows / factor, depth_im.cols / factor);
    for_all_rc (subsampled_im)
        subsampled_im (r, c) = depth_im (r * factor, c * factor);
}

void subsampleDepthSmooth (const cv::Mat1f& depth_im, cv::Mat1f& subsampled_im, int factor, float max_depth_diff)
{
    subsampled_im.create (depth_im.rows / factor, depth_im.cols / factor);

    for_all_rc (subsampled_im)
    {
        float center_depth = depth_im (r * factor, c * factor);

        if (center_depth < 1e-5)
        {
            subsampled_im (r, c) = 1e-5;
            continue;
        }

        if (!(r > 2 && r < (subsampled_im.rows - 2) && c > 2 && c < (subsampled_im.cols - 2)))
        {
            subsampled_im (r, c) = center_depth;
            continue;
        }

        int count = 0;
        float sum = 0.f;

        for (int dy = -2; dy < 3; ++dy)
        for (int dx = -2; dx < 3; ++dx)
        {
            float new_depth = depth_im (r * factor + dy, c * factor + dx);
            if (new_depth > 1e-5 && std::abs(new_depth - center_depth) < max_depth_diff)
            {
                ++count;
                sum += new_depth;
            }
        }

        subsampled_im (r, c) = sum / count;
    }
}

void computeNormalsEigen (const cv::Mat1f& depth_im, const ntk::Pose3D& depth_pose, cv::Mat3f& normals_im)
{
#ifdef NESTK_USE_PCL
    ntk_ensure (depth_pose.isValid(), "Calibration required.");

    ntk::TimeCount tc_normals("Normals", 1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointXYZ nan_point;
    nan_point.getVector4fMap().setConstant (std::numeric_limits<float>::quiet_NaN());

    cloud->width = depth_im.cols;
    cloud->height = depth_im.rows;
    cloud->points.resize(cloud->width*cloud->height, nan_point);
    cloud->is_dense = true;

    for (int r = 0; r < depth_im.rows; ++r)
    for (int c = 0; c < depth_im.cols; ++c)
    {
        float d = depth_im (r,c);
        if (d < 1e-5)
            continue;

        cv::Point3f p = depth_pose.unprojectFromImage(cv::Point2f(c,r), d);
        pcl::PointXYZ pcl_p;
        pcl_p.x = p.x;
        pcl_p.y = p.y;
        pcl_p.z = p.z;
        cloud->points[r*cloud->width+c] = pcl_p;
    }

    tc_normals.elapsedMsecs(" -- imageToPointCloud");

    pcl::PointCloud<pcl::Normal> normals;
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setNormalEstimationMethod (normal_estimator.COVARIANCE_MATRIX);
    normal_estimator.setMaxDepthChangeFactor (1.f);
    normal_estimator.setNormalSmoothingSize (7.0f);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.compute(normals);
    tc_normals.elapsedMsecs(" -- compute normals");

    normals_im.create (depth_im.size());
    fillWithNan (normals_im);
    for_all_rc (depth_im)
    {
        pcl::Normal& p = normals.points[depth_im.cols*r+c];
        if (p.normal_z < 0) continue;
        cv::Vec3f cv_p (p.normal_x, p.normal_y, p.normal_z);
        normalize(cv_p);
        ntk_assert(cv::norm(cv_p) > 0.9 || ntk_isnan(cv_p[0]), "No null normal");
        normals_im (r,c) = depth_pose.invCameraTransform(cv_p); // normals are in image space.
    }

    tc_normals.stop(" -- transformation to Mat1f");
#endif
}

void computeNormals (const cv::Mat1f& depth_im, const ntk::Pose3D& depth_pose, cv::Mat3f& normals_im)
{
    ntk_ensure (depth_pose.isValid(), "Calibration required.");
    ntk::TimeCount tc ("computeNormals", 2);

    normals_im.create (depth_im.size());
    normals_im = infinite_point();

    cv::Mat1f dx, dy;

    // norm is 32 for 3x3 kernel.
    // norm is 128 for 5x5 kernel.
    float kernel_norm = 1.0/128.0; // sobel 5x5 multiplies by 128 the unit derivate.
    cv::Sobel(depth_im, dx, CV_32F, 1, 0, 5, kernel_norm);
    cv::Sobel(depth_im, dy, CV_32F, 0, 1, 5, kernel_norm);

    for_all_rc(depth_im)
    {
        cv::Vec3f n = estimate_normal_from_depth (depth_im, depth_pose,
                                                  r, c, 0.03f,
                                                  dx.data ? &dx : 0, dy.data ? &dy : 0);
        normals_im (r,c) = n;
    }
    tc.stop();
}

} // ntk

namespace ntk {

static void copy16bitsToFloat (const cv::Mat1w& src_im, cv::Mat1f& dest_im, float raw_depth_unit_in_meters)
{
    dest_im = cv::Mat1f (src_im.size ());
    const uint16_t* src = src_im.ptr<uint16_t>();
    const uint16_t* end = src + src_im.cols * src_im.rows;
    float* output = dest_im.ptr<float>();
    while (end != src)
        *output++ = (*src++) * raw_depth_unit_in_meters;
}

SoftKineticRGBDProcessor::SoftKineticRGBDProcessor()
    : RGBDProcessor()
{
    setMinDepth (0.15f);
    setMaxDepth (2.0f);
    setFilterFlags(RGBDProcessorFlags::FilterMedian | RGBDProcessorFlags::FilterEdges /* | RGBDProcessorFlags::ErodeDepthBorders */);
    // setFilterFlags(RGBDProcessorFlags::FilterBilateral | RGBDProcessorFlags::FilterEdges);
}

Kin4winRGBDProcessor::Kin4winRGBDProcessor()
    : OpenniRGBDProcessor()
{
    // Everything is done by the grabber.
    setFilterFlags(RGBDProcessorFlags::NiteProcessed | RGBDProcessorFlags::ComputeMapping);
}

void Kin4winRGBDProcessor::computeMappings()
{
    if (m_image->calibration().empty())
        return;

    if (m_image->calibration()->depth_pose->isIdentity()
            && m_image->calibration()->rgb_pose->isIdentity())
    {
        // Dummy mapping, like when using Openni registration.
        // ntk_dbg(1) << "Using OPENNI rgbd processor mapping.";
        return OpenniRGBDProcessor::computeMappings();
    }
    else
    {
        // Do the actual mapping using transforms.
        // ntk_dbg(1) << "Using rgbd processor mapping.";
        return RGBDProcessor::computeMappings();
    }
}

} // ntk

namespace ntk
{

    RGBDProcessor :: RGBDProcessor() :
            m_image(0),
            //m_flags(FixGeometry | FixBias | UndistortImages | ComputeNormals),
            m_flags(RGBDProcessorFlags::UndistortImages),
            m_min_depth(0.1f),
            m_max_depth(10.0f),
            m_max_normal_angle(80),
            m_max_time_depth_delta(0.1f),
            m_max_spatial_depth_delta(0.05f),
            m_mapping_resolution(1.0f),
            m_min_amplitude(1000),
            m_max_amplitude(-1)
    {
    }

    void RGBDProcessor :: fixDepthGeometry()
    {
        ntk_ensure(m_image->calibration(), "Calibration required.");
        double cx = m_image->calibration()->depth_intrinsics(0,2);
        double cy = m_image->calibration()->depth_intrinsics(1,2);
        double fx = m_image->calibration()->depth_intrinsics(0,0);
        double fy = m_image->calibration()->depth_intrinsics(1,1);

        cv::Mat1f& depth_im = m_image->depthRef();
        for_all_rc(depth_im)
        {
            double orig_depth = depth_im(r,c);

            double dx = c-cx;
            double dy = r-cy;

            cv::Point3f v(dx/fx, dy/fy, 1);
            double norm = sqrt(v.dot(v));
            v = v * (1.0/norm);
            v *= orig_depth;

            double depth_z = v.z;
            depth_im(r,c) = depth_z;
        }
    }

    void RGBDProcessor :: fixDepthBias()
    {
        ntk_assert(0, "not updated.");
#if 0 // FIXME: obsolete
        cv::Mat1f& depth_im = m_image->depthRef();
        for_all_rc(depth_im)
        {
            double depth_z = depth_im(r,c);

            // depth correction is accurate only in the 30cm-110cm range.
            double x = depth_z;
            if (depth_z < 0.3) x = 0.3;
            if (depth_z > 0.8) x = 0.8;
            double offset =   m_image->calibration()->depth_correction(0,4)*x*x*x*x
                              + m_image->calibration()->depth_correction(0,3)*x*x*x
                              + m_image->calibration()->depth_correction(0,2)*x*x
                              + m_image->calibration()->depth_correction(0,1)*x
                              + m_image->calibration()->depth_correction(0,0);

            depth_im(r,c) = depth_z + offset;
        }
#endif
    }

    // FIXME: why is it so slow ?
    void RGBDProcessor :: computeNormals(RGBDImage& image)
    {
        ntk_ensure (image.calibration(), "Calibration required.");
        ntk::computeNormals (image.depth(), *image.calibration()->depth_pose, image.normalRef());
    }

#ifdef NESTK_USE_PCL
    void RGBDProcessor :: computeNormalsPCL(RGBDImage& image)
    {
        ntk_ensure(image.calibration(), "Calibration required.");

        ntk::TimeCount tc_normals("Normals", 1);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        rgbdImageToPointCloud(*cloud, image, true /* keep dense */);
        tc_normals.elapsedMsecs(" -- imageToPointCloud");

        pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::PointCloud<pcl::Normal> normals;

        // ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
        ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
        ne.setMaxDepthChangeFactor(0.1f);
        ne.setNormalSmoothingSize(5.0f);
        ne.setInputCloud(cloud); // FIXME: Find out whether the removal of the (deep-copying) cloud.makeShared() call sped things up.
        ne.compute(normals);
        tc_normals.elapsedMsecs(" -- compute normals");

        const Pose3D& depth_pose = *image.calibration()->depth_pose;
        const cv::Mat1f& depth_im = image.depth();
        cv::Mat3f& normal_im = image.normalRef();
        normal_im = cv::Mat3f(depth_im.size());
        fillWithNan(normal_im);
        for_all_rc(depth_im)
        {
            pcl::Normal& p = normals.points[depth_im.cols*r+c];
            if (p.normal_z < 0) continue;
            cv::Vec3f cv_p (p.normal_x, p.normal_y, p.normal_z);
            normalize(cv_p);
            ntk_assert(cv::norm(cv_p) > 0.9 || ntk_isnan(cv_p[0]), "No null normal");
            normal_im(r,c) = depth_pose.invCameraTransform(cv_p); // normals are in image space.
        }

        tc_normals.stop(" -- transformation to Mat1f");
    }

    void RGBDProcessor :: computeHighQualityNormalsPCL(RGBDImage& image)
    {
        ntk_ensure(image.calibration(), "Calibration required.");

        ntk::TimeCount tc_normals("Normals", 1);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        rgbdImageToPointCloud(*cloud, image, true /* is_dense */);
        tc_normals.elapsedMsecs(" -- imageToPointCloud");

        pcl::PointCloud<pcl::Normal> normals;
        pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setNormalEstimationMethod (normal_estimator.COVARIANCE_MATRIX);
        normal_estimator.setMaxDepthChangeFactor (0.05f);
        normal_estimator.setNormalSmoothingSize (20.0f);
        normal_estimator.setInputCloud(cloud);
        normal_estimator.compute(normals);
        tc_normals.elapsedMsecs(" -- compute normals");

        const Pose3D& depth_pose = *image.calibration()->depth_pose;
        const cv::Mat1f& depth_im = image.depth();
        cv::Mat3f& normal_im = image.normalRef();
        normal_im = cv::Mat3f(depth_im.size());
        fillWithNan(normal_im);
        for_all_rc(depth_im)
        {
            pcl::Normal& p = normals.points[depth_im.cols*r+c];
            if (p.normal_z < 0) continue;
            cv::Vec3f cv_p (p.normal_x, p.normal_y, p.normal_z);
            normalize(cv_p);
            ntk_assert(cv::norm(cv_p) > 0.9 || ntk_isnan(cv_p[0]), "No null normal");
            normal_im(r,c) = depth_pose.invCameraTransform(cv_p); // normals are in image space.
        }

        tc_normals.stop(" -- transformation to Mat1f");
    }
#else
    void RGBDProcessor :: computeNormalsPCL(RGBDImage& image)
    {
        ntk_assert(0, "PCL support not enabled.");
    }

    void RGBDProcessor :: computeHighQualityNormalsPCL(RGBDImage& image)
    {
        ntk_assert(0, "PCL support not enabled.");
    }
#endif

    void RGBDProcessor :: processImage(RGBDImage& image)
    {
        m_image = &image;

        TimeCount tc("processImage", 2);

        if (m_image->hasRgb() && hasFilterFlag(RGBDProcessorFlags::FlipColorImage))
        {
            flip(m_image->rawRgb(), m_image->rawRgbRef(), -1);
        }

        if (!hasFilterFlag(RGBDProcessorFlags::NiteProcessed) && m_image->rawIntensity().data)
        {
            normalize(m_image->rawIntensityRef(), m_image->rawIntensityRef(), 0, 255, cv::NORM_MINMAX, -1);
        }

        if (hasFilterFlag(RGBDProcessorFlags::NiteProcessed) || (!m_image->calibration() || !hasFilterFlag(RGBDProcessorFlags::UndistortImages) || hasFilterFlag(RGBDProcessorFlags::Pause)))
        {
            m_image->rawAmplitude().copyTo(m_image->amplitudeRef());
            if (!m_image->rawDepth().empty())
            {
                m_image->rawDepth().copyTo(m_image->depthRef());
            }
            else
            {
                const float unit_in_meters = m_image->calibration() ? m_image->calibration()->rawDepthUnitInMeters() : 1.f/1000.f;
                copy16bitsToFloat (m_image->rawDepth16bits(), m_image->depthRef(), unit_in_meters);
            }
            m_image->rawRgb().copyTo(m_image->rgbRef());
            m_image->rawIntensity().copyTo(m_image->intensityRef());
        }

        if (m_image->rawDepth().empty() && m_image->rawDepth16bits().empty())
            return;

        if (!hasFilterFlag(RGBDProcessorFlags::NiteProcessed) && m_image->calibration() && hasFilterFlag(RGBDProcessorFlags::UndistortImages))
            undistortImages();
        tc.elapsedMsecs("undistort");

        if (hasFilterFlag(RGBDProcessorFlags::ComputeKinectDepthLinear))
            computeKinectDepthLinear();
        else if (hasFilterFlag(RGBDProcessorFlags::ComputeKinectDepthTanh))
            computeKinectDepthTanh();
        else if (hasFilterFlag(RGBDProcessorFlags::ComputeKinectDepthBaseline))
            computeKinectDepthBaseline();
        tc.elapsedMsecs("computeDepth");

        m_image->depthMaskRef() = cv::Mat1b(m_image->depth().size());
        m_image->header().filter_min_depth = m_min_depth;
        m_image->header().filter_max_depth = m_max_depth;
        for_all_rc(m_image->depthMaskRef())
        {
            float d = m_image->depth()(r,c);
            if (d < m_min_depth || d > m_max_depth || ntk::math::isnan(d))
                m_image->depthMaskRef()(r,c) = 0;
            else
                m_image->depthMaskRef()(r,c) = 255;
        }

        if (m_image->calibration())
        {
            if (!flt_eq(m_image->calibration()->depth_multiplicative_correction_factor, 1.0f, 1e-5f)
                    || !flt_eq(m_image->calibration()->depth_additive_correction_factor, 0.0f, 1e-5f))
            {
                float factor = m_image->calibration()->depth_multiplicative_correction_factor;
                float offset = m_image->calibration()->depth_additive_correction_factor;
                ntk_dbg_print(m_image->calibration()->depth_multiplicative_correction_factor, 1);
                ntk_dbg_print(m_image->calibration()->depth_additive_correction_factor, 1);
                cv::Mat1f& depth_im = m_image->depthRef();
                for_all_rc(depth_im)
                {
                    float& d = depth_im.at<float>(r,c);
                    if (d > 1e-5)
                        d = d*factor + offset;
                }
            }

            if (hasFilterFlag(RGBDProcessorFlags::FixGeometry))
                fixDepthGeometry();

            if (hasFilterFlag(RGBDProcessorFlags::FixBias))
                fixDepthBias();

            if (hasFilterFlag(RGBDProcessorFlags::RemoveSmallStructures))
                removeSmallStructures();

            if (hasFilterFlag(RGBDProcessorFlags::FilterMedian))
                medianFilter();

            if (hasFilterFlag(RGBDProcessorFlags::FilterBilateral))
                bilateralFilter(*m_image);

            tc.elapsedMsecs("bilateralFilter");

            if (hasFilterFlag(RGBDProcessorFlags::ComputeNormals) || hasFilterFlag(RGBDProcessorFlags::FilterNormals))
            {
                if (hasFilterFlag(RGBDProcessorFlags::ComputeHighQualityNormals))
                    computeHighQualityNormalsPCL(*m_image);
                else
                    computeNormals(*m_image);
            }

            tc.elapsedMsecs("computeNormals");

            if (hasFilterFlag(RGBDProcessorFlags::FilterThresholdDepth))
                applyDepthThreshold();

            tc.elapsedMsecs("depthThreshold");

            if (hasFilterFlag(RGBDProcessorFlags::FilterAmplitude))
                removeLowAmplitudeOutliers();

            if (hasFilterFlag(RGBDProcessorFlags::FilterEdges))
                removeEdgeOutliers();

            if (hasFilterFlag(RGBDProcessorFlags::FilterNormals))
                removeNormalOutliers();

            if (hasFilterFlag(RGBDProcessorFlags::FilterUnstable))
                removeUnstableOutliers();

            tc.elapsedMsecs("unstableOutliers");

            if (hasFilterFlag(RGBDProcessorFlags::ComputeMapping))
                computeMappings();

            tc.elapsedMsecs("computeMappings");

            if (hasFilterFlag(RGBDProcessorFlags::FillSmallHoles))
                fillSmallHoles();

            if (hasFilterFlag(RGBDProcessorFlags::ErodeDepthBorders))
                erodeDepthBorders();
        }

        if (m_image->rgb().data)
            cvtColor(m_image->rgb(), m_image->rgbAsGrayRef(), CV_BGR2GRAY);
        tc.stop();
    }

    void RGBDProcessor :: undistortImages()
    {
        cv::Mat3b tmp3b;
        cv::Mat1f tmp1f;

        cv::Mat3b& rgb_im = m_image->rgbRef();
        const cv::Mat3b& raw_rgb_im = m_image->rawRgb();

        if (m_image->calibration()->rgbSize() != m_image->calibration()->rawRgbSize())
        {
            // First cut color image to the undistorted image size (not used for kinect).
            cv::Size rgb_size = m_image->calibration()->rgbSize();
            cv::Mat roi = raw_rgb_im(cv::Rect((raw_rgb_im.cols-rgb_size.width)/2.0,
                                              (raw_rgb_im.rows-rgb_size.height)/2.0,
                                              rgb_size.width,
                                              rgb_size.height));
            roi.copyTo(rgb_im);
        }
        else
        {
            raw_rgb_im.copyTo(rgb_im);
        }

        if (!m_image->calibration()->zero_rgb_distortion)
        {
            if (m_image->calibration()->rgb_undistort_map1.empty())
                const_Ptr_cast<RGBDCalibration>(m_image->calibration())->updateDistortionMaps();
            remap(rgb_im, tmp3b,
                  m_image->calibration()->rgb_undistort_map1,
                  m_image->calibration()->rgb_undistort_map2,
                  CV_INTER_LINEAR);
            tmp3b.copyTo(rgb_im);
        }

        if (!m_image->calibration()->zero_depth_distortion)
        {
            if (m_image->calibration()->depth_undistort_map1.empty())
                const_Ptr_cast<RGBDCalibration>(m_image->calibration())->updateDistortionMaps();
            remap(m_image->rawDepthRef(), m_image->depthRef(),
                  m_image->calibration()->depth_undistort_map1,
                  m_image->calibration()->depth_undistort_map2,
                  CV_INTER_LINEAR);
            ntk_assert(m_image->depth().data != 0, "Should be ok");
        }
        else
        {
            if (!m_image->rawDepthRef().empty())
                m_image->rawDepthRef().copyTo(m_image->depthRef());
            else
            {
                const float unit_in_meters = m_image->calibration() ? m_image->calibration()->rawDepthUnitInMeters() : 1.f/1000.f;
                copy16bitsToFloat (m_image->rawDepth16bits(), m_image->rawDepthRef(), unit_in_meters);
            }
        }

        if (m_image->calibration()->zero_depth_distortion ||
            hasFilterFlag(RGBDProcessorFlags::NoAmplitudeIntensityUndistort))
        {
            m_image->rawAmplitudeRef().copyTo(m_image->amplitudeRef());
            m_image->rawIntensityRef().copyTo(m_image->intensityRef());
        }
        else
        {
            if (m_image->rawAmplitudeRef().data)
            {
                remap(m_image->rawAmplitudeRef(), m_image->amplitudeRef(),
                      m_image->calibration()->depth_undistort_map1,
                      m_image->calibration()->depth_undistort_map2,
                      CV_INTER_LINEAR);
            }

            if (m_image->rawIntensityRef().data)
            {
                remap(m_image->rawIntensityRef(), m_image->intensityRef(),
                      m_image->calibration()->depth_undistort_map1,
                      m_image->calibration()->depth_undistort_map2,
                      CV_INTER_LINEAR);
            }
        }
    }

    void RGBDProcessor :: computeMappings()
    {
        ntk_ensure(m_image->calibration(), "Calibration required.");
        ntk_ensure(m_image->hasRgb(), "Image does not have rgb data!");

        cv::Mat1b& mask_im = m_image->depthMaskRef();

        const Pose3D& depth_pose = *m_image->calibration()->depth_pose;
        const Pose3D& rgb_pose = *m_image->calibration()->rgb_pose;

        const cv::Mat1f& depth_im = m_image->depth();
        const cv::Mat3b& rgb_im = m_image->rgb();

        cv::Mat1f& mapped_depth = m_image->mappedDepthRef();
        mapped_depth = cv::Mat1f(m_image->rgb().size());
        mapped_depth = 0.f;

        cv::Mat3b& mapped_color = m_image->mappedRgbRef();
        mapped_color = cv::Mat3b(m_image->depth().size());
        mapped_color = cv::Vec3b(0,0,0);

        float delta = 1.0 / m_mapping_resolution;
        for (float r = 0; r < depth_im.rows; r += delta )
            for (float c = 0; c < depth_im.cols; c += delta )
            {
            int i_r = ntk::math::rnd(r);
            int i_c = ntk::math::rnd(c);
            if (!is_yx_in_range(depth_im, i_r, i_c))
                continue;

            if (!mask_im(i_r, i_c))
                continue;

            double depth = depth_im(i_r,i_c);
            cv::Point3f p = depth_pose.unprojectFromImage(cv::Point2f(c,r), depth);
            cv::Point3f prgb = rgb_pose.projectToImage(p);

            int i_y = ntk::math::rnd(prgb.y);
            int i_x = ntk::math::rnd(prgb.x);
            if (is_yx_in_range(rgb_im, i_y, i_x))
            {
                cv::Vec3b bgr = rgb_im(i_y, i_x);
                mapped_color(i_r, i_c) = bgr;
                mapped_depth(i_y, i_x) = prgb.z;
            }
        }
    }

    void RGBDProcessor :: medianFilter()
    {
        medianBlur(m_image->depthRef(), m_image->depthRef(), 3);
    }

    void RGBDProcessor :: bilateralFilter(RGBDImage& image)
    {
        cv::Mat1f tmp;
        depth_bilateralFilter(image.depthRef(), tmp, 7, 20, 20, 0.01f);
        tmp.copyTo(image.depthRef());
    }

#if 0
    void RGBDProcessor :: removeLowAmplitudeOutliers()
    {
        ntk_ensure(m_image->calibration(), "Calibration required.");
        cv::Mat1b& mask_im = m_image->depthMaskRef();
        const cv::Mat1f& depth_im = m_image->depth();
        double amplitude_sum = sum(m_image->amplitude())[0];

        for_all_rc(depth_im)
        {
            if (!mask_im(r,c)) continue;
            double da = m_image->amplitude()(r,c) / amplitude_sum;
            if (da < 0.02)
                mask_im(r,c) = 0;
        }
    }
#else
    void RGBDProcessor :: removeLowAmplitudeOutliers()
    {
        cv::Mat1b& mask_im = m_image->depthMaskRef();
        const cv::Mat1f& depth_im = m_image->depth();

        for_all_rc(depth_im)
        {
            if (!mask_im(r,c)) continue;
            double a = m_image->amplitude()(r,c);

            bool keep_it = true;
            keep_it &= (m_min_amplitude < 0 || a > m_min_amplitude);
            keep_it &= (m_max_amplitude < 0 || a < m_max_amplitude);

            if (!keep_it)
                mask_im(r,c) = 0;
        }
    }
#endif

    void RGBDProcessor :: removeNormalOutliers()
    {
        ntk_ensure(m_image->calibration(), "Calibration required.");
        const Pose3D& depth_pose = *m_image->calibration()->depth_pose;
        cv::Mat1b& mask_im = m_image->depthMaskRef();
        const cv::Mat1f& depth_im = m_image->depth();

        for_all_rc(depth_im)
        {
            if (!mask_im(r,c)) continue;
            if (!m_image->isValidNormal(r,c)) continue;
            cv::Vec3f eyev = camera_eye_vector(depth_pose, r, c);
            cv::Vec3f normal = m_image->normal()(r, c);
            double angle = acos(normal.dot(-eyev));
            if (angle > (m_max_normal_angle*M_PI/180.0))
                mask_im(r,c) = 0;
        }
    }

    void RGBDProcessor :: removeUnstableOutliers()
    {
        ntk_ensure(m_image->calibration(), "Calibration required.");
        cv::Mat1b& mask_im = m_image->depthMaskRef();
        const cv::Mat1f& depth_im = m_image->depth();

        if (!m_last_depth_image.data)
        {
            m_image->depth().copyTo(m_last_depth_image);
            return;
        }

        for_all_rc(depth_im)
        {
            if (!mask_im(r,c)) continue;
            float diff = std::abs(m_last_depth_image(r,c) - depth_im(r,c));
            if (diff > m_max_time_depth_delta)
                mask_im(r,c) = 0;
        }

        m_image->depth().copyTo(m_last_depth_image);
    }

    void RGBDProcessor :: removeSmallStructures()
    {
        ntk_ensure(m_image->calibration(), "Calibration required.");
        cv::Mat1b& mask_im = m_image->depthMaskRef();

        cv::Mat1b tmp;

        cv::morphologyEx(mask_im, tmp,
                         cv::MORPH_CLOSE,
                         getStructuringElement(cv::MORPH_CROSS,
                                               cv::Size(3,3)));

        cv::morphologyEx(tmp, mask_im,
                         cv::MORPH_OPEN,
                         getStructuringElement(cv::MORPH_CROSS,
                                               cv::Size(3,3)));

    }

    void RGBDProcessor :: fillSmallHoles()
    {
        ntk_ensure(m_image->calibration(), "Calibration required.");
        cv::Mat1b& mask_im = m_image->depthMaskRef();
        cv::Mat1f& depth_im = m_image->depthRef();

        cv::Mat1b new_mask;
        cv::morphologyEx(mask_im, new_mask,
                         cv::MORPH_CLOSE,
                         getStructuringElement(cv::MORPH_RECT,
                                               cv::Size(10,10)));
        cv::Mat1b paint_mask = new_mask - mask_im;
        mask_im = new_mask;

        for_all_rc(mask_im)
        {
            if (mask_im(r,c) && depth_im(r,c) < 1e-5)
                paint_mask(r,c) = 1;
        }

        cv::Mat1b depth8b (depth_im.size());
        for_all_rc(depth8b)
        {
            depth8b(r,c) = 255 * (depth_im(r,c) - m_min_depth) / (m_max_depth - m_min_depth);
        }

        cv::Mat1b painted_depth8b;

        cv::inpaint(depth8b, paint_mask, painted_depth8b, 10, cv::INPAINT_NS);
        imwrite("debug_depth.png", depth8b);
        imwrite("debug_painted_depth.png", painted_depth8b);
        imwrite("debug_paint_mask.png", paint_mask);
        imwrite("debug_depth_mask.png", mask_im);

        for_all_rc(paint_mask)
        {
            if (paint_mask(r,c))
            {
                double d = painted_depth8b(r,c) * (m_max_depth-m_min_depth) / 255.0 + m_min_depth;
                depth_im(r,c) = d;
            }
        }
    }

    void RGBDProcessor::erodeDepthBorders()
    {
        cv::Mat1b& depth_mask_im = m_image->depthMaskRef();
        cv::Mat1b tmp_im;
        depth_mask_im.copyTo(tmp_im);
        cv::morphologyEx(tmp_im, depth_mask_im,
                         cv::MORPH_ERODE,
                         getStructuringElement(cv::MORPH_RECT,
                                               cv::Size(3,3)));
    }

    void RGBDProcessor :: removeEdgeOutliers()
    {
        ntk_ensure(m_image->calibration(), "Calibration required.");
        cv::Mat1b& mask_im = m_image->depthMaskRef();
        const cv::Mat1f& depth_im = m_image->depth();

        for_all_rc(depth_im)
        {
            if (!mask_im(r,c)) continue;

            if (!is_yx_in_range(depth_im, r+1, c+1))
                continue;
            double diff = std::abs(depth_im(r,c) - depth_im(r,c+1))
                          + std::abs(depth_im(r,c) - depth_im(r+1,c));
            diff /= 2.0;
            if (diff > m_max_spatial_depth_delta)
            {
                mask_im(r,c) = 0;
                continue;
            }
        }
    }

    void RGBDProcessor :: computeKinectDepthTanh()
    {
        const float k1 = 1.1863f;
        const float k2 = 2842.5f;
        const float k3 = 0.1236f;

        cv::Mat1f& depth_im = m_image->depthRef();
        for_all_rc(depth_im)
        {
            float depth = depth_im(r,c);
            if (depth < 2047)
            {
                depth = k3 * tanf(depth/k2 + k1);
            }
            else
                depth = 0;
            depth_im(r,c) = depth;
        }
    }

    void RGBDProcessor :: computeKinectDepthBaseline()
    {
        cv::Mat1f& depth_im = m_image->depthRef();

        double depth_baseline = 7.5e-02;
        double depth_offset = 1090;
        double focal_ir = 580;
        if (m_image->calibration())
        {
            depth_baseline = m_image->calibration()->depth_baseline;
            depth_offset = m_image->calibration()->depth_offset;
            focal_ir = m_image->calibration()->depth_pose->meanFocal();
        }

        for_all_rc(depth_im)
        {
            float raw_depth = depth_im(r,c);
            float depth = 0;
            if (raw_depth < 2047)
            {
                depth = focal_ir * 8.0 * depth_baseline / (depth_offset - raw_depth);
            }
            if (depth < 0)
                depth = 0;
            else if (depth > 30)
                depth = 30;
            depth_im(r,c) = depth;
        }
    }

    void RGBDProcessor :: computeKinectDepthLinear()
    {
        cv::Mat1f& depth_im = m_image->depthRef();
        for_all_rc(depth_im)
        {
            float raw_depth = depth_im(r,c);
            float depth = 0;
            if (raw_depth < 2047)
            {
                depth = 1.0 / (raw_depth * -0.0030711016 + 3.3309495161);
            }
            if (depth < 0)
                depth = 0;
            else if (depth > 30)
                depth = 30;
            depth_im(r,c) = depth;
        }
    }

    void RGBDProcessor :: applyDepthThreshold()
    {
        cv::Mat1b& mask_im = m_image->depthMaskRef();
        const cv::Mat1f& depth_im = m_image->depth();

        for_all_rc(depth_im)
        {
            float depth = depth_im(r,c);
            if (depth < m_min_depth || depth > m_max_depth)
                mask_im(r,c) = 0;
        }
    }

    void DepthVisualizer::buildLookup()
    {
        for (int v = 0; v < 255*6+1; ++v)
        {
            unsigned char r,g,b;
            int lb = v & 0xff;
            switch (v / 256) {
            case 0:
                r = 255;
                g = 255-lb;
                b = 255-lb;
                break;
            case 1:
                r = 255;
                g = lb;
                b = 0;
                break;
            case 2:
                r = 255-lb;
                g = 255;
                b = 0;
                break;
            case 3:
                r = 0;
                g = 255;
                b = lb;
                break;
            case 4:
                r = 0;
                g = 255-lb;
                b = 255;
                break;
            case 5:
                r = 0;
                g = 0;
                b = 255-lb;
                break;
            default:
                r = 0;
                g = 0;
                b = 0;
                break;
            }
            if (v == 0)
            {
                r = g = b = 0;
            }
            color_lookup[v] = cv::Vec3b(b,g,r);
        }
    }

    void DepthVisualizer::computeColorEncoded(const cv::Mat1f& depth_im, cv::Mat3b& color_depth_im,
                                              double* i_min_val, double* i_max_val)
    {
        double min_val, max_val;
        if (i_min_val && i_max_val)
        {
            min_val = *i_min_val;
            max_val = *i_max_val;
        }
        else
        {
            minMaxLoc(depth_im, &min_val, &max_val);
        }

        color_depth_im.create(depth_im.size());

#if 1
        const float* depth_data = depth_im.ptr<float>(0);
        const float* end_depth_data = depth_data + depth_im.rows*depth_im.cols;
        cv::Vec3b* depth_color_data = color_depth_im.ptr<cv::Vec3b>(0);
        const float norm = 1.f/(max_val-min_val);
        while (depth_data != end_depth_data)
        {
            int v = 255*6*(*depth_data-min_val)*norm;
            v = ntk::saturate_to_range(v, 0, 255*6);
            *depth_color_data = color_lookup[v];
            ++depth_color_data;
            ++depth_data;
        }
#else
        for (int r = 0; r < depth_im.rows; ++r)
        {
            const float* depth_data = depth_im.ptr<float>(r);
            cv::Vec3b* depth_color_data = color_depth_im.ptr<cv::Vec3b>(r);
            for (int c = 0; c < depth_im.cols; ++c)
            {
                int v = 255*6*(depth_data[c]-min_val)/(max_val-min_val);
                depth_color_data[c] = color_lookup[v];
            }
        }
#endif
    }
	  
    void compute_color_encoded_depth(const cv::Mat1f& depth_im, cv::Mat3b& color_depth_im,
                                     double* i_min_val, double* i_max_val)
    {
        double min_val, max_val;
        if (i_min_val && i_max_val)
        {
            min_val = *i_min_val;
            max_val = *i_max_val;
        }
        else
        {
            minMaxLoc(depth_im, &min_val, &max_val);
        }

        color_depth_im.create(depth_im.size());
        for (int r = 0; r < depth_im.rows; ++r)
        {
            const float* depth_data = depth_im.ptr<float>(r);
            cv::Vec3b* depth_color_data = color_depth_im.ptr<cv::Vec3b>(r);
            for (int c = 0; c < depth_im.cols; ++c)
            {
                int v = 255*6*(depth_data[c]-min_val)/(max_val-min_val);
                if (v < 0) v = 0;
                unsigned char r,g,b;
                int lb = v & 0xff;
                switch (v / 256) {
                case 0:
                    r = 255;
                    g = 255-lb;
                    b = 255-lb;
                    break;
                case 1:
                    r = 255;
                    g = lb;
                    b = 0;
                    break;
                case 2:
                    r = 255-lb;
                    g = 255;
                    b = 0;
                    break;
                case 3:
                    r = 0;
                    g = 255;
                    b = lb;
                    break;
                case 4:
                    r = 0;
                    g = 255-lb;
                    b = 255;
                    break;
                case 5:
                    r = 0;
                    g = 0;
                    b = 255-lb;
                    break;
                default:
                    r = 0;
                    g = 0;
                    b = 0;
                    break;
                }
                if (v == 0)
                {
                    r = g = b = 0;
                }
                depth_color_data[c] = cv::Vec3b(b,g,r);
            }
        }
    }

    OpenniRGBDProcessor::OpenniRGBDProcessor()
        : RGBDProcessor()
    {
        // Everything is done by the grabber.
        setFilterFlags(RGBDProcessorFlags::NiteProcessed
                       | RGBDProcessorFlags::ComputeMapping/*
                       | RGBDProcessorFlags::ErodeDepthBorders
                       | RGBDProcessorFlags::RemoveSmallStructures*/);
    }

    void OpenniRGBDProcessor :: computeMappings()
    {
        cv::Size depth_size = m_image->calibration()->raw_depth_size;
        cv::Size rgb_size = m_image->calibration()->rawRgbSize();

        bool mapping_required = depth_size != rgb_size;
        if (!mapping_required)
        {
            m_image->mappedDepthRef() = m_image->depth();
            m_image->mappedRgbRef() = m_image->rgb();
            m_image->mappedDepthMaskRef() = m_image->depthMask();
            return;
        }
        else
        {
            m_image->mappedRgbRef().create(m_image->calibration()->raw_depth_size);
            m_image->mappedDepthRef().create(m_image->calibration()->rawRgbSize());
            m_image->mappedDepthMaskRef().create(m_image->calibration()->rawRgbSize());
        }

        const float ratio = float(rgb_size.width) / depth_size.width;

        cv::Size raw_rgb_size(rgb_size.width/ratio, rgb_size.height/ratio);
        cv::Mat3b raw_mapped_rgb(raw_rgb_size);
        cv::resize(m_image->rawRgb(), raw_mapped_rgb, raw_rgb_size, 0, 0, cv::INTER_LINEAR);
        m_image->mappedRgbRef() = raw_mapped_rgb(cv::Rect(cv::Point(0,0),
                                                      depth_size));

        cv::Size raw_depth_size(depth_size.width*ratio, depth_size.height*ratio);

        cv::Mat1f mapped_depth(raw_depth_size);
        cv::resize(m_image->depth(), mapped_depth, raw_depth_size, 0, 0, cv::INTER_NEAREST);

        cv::Mat1b raw_mapped_depth_mask(raw_depth_size);
        cv::resize(m_image->depthMask(), raw_mapped_depth_mask, raw_depth_size, 0, 0, cv::INTER_NEAREST);

        cv::Mat1f roi_depth = m_image->mappedDepthRef()(cv::Rect(cv::Point(0,0), raw_depth_size));
        mapped_depth.copyTo(roi_depth);

        cv::Mat1b roi_depth_mask = m_image->mappedDepthMaskRef()(cv::Rect(cv::Point(0,0), raw_depth_size));
        raw_mapped_depth_mask.copyTo(roi_depth_mask);
    }

} // ntk

namespace ntk
{

    RGBDProcessor* RGBDProcessorFactory :: createProcessor(const RGBDProcessorFactory::Params& params)
    {
        RGBDProcessor* processor = 0;
        if (params.grabber_type == "openni" || params.grabber_type == "openni2")
        {
            processor = new OpenniRGBDProcessor();
        }
        else if (params.grabber_type == "kin4win")
        {
            processor = new Kin4winRGBDProcessor();
        }
        else if (params.grabber_type == "freenect")
        {
            processor = new FreenectRGBDProcessor();
        }
        else if (params.grabber_type == "softkinetic")
        {
            ntk_dbg(1) << "Creating a softkinetic processor";
            processor = new SoftKineticRGBDProcessor();
        }
#ifdef NESTK_USE_PMDSDK
        else if (params.grabber_type == "pmd")
        {
            processor = new PmdRGBDProcessor();
        }
#endif
        else
        {
            // By default
            ntk_dbg(0) << "Warning: don't know which rgbd processor to create for " << params.grabber_type << " , creating OpenNI.";
            processor = new OpenniRGBDProcessor();
        }

        if (params.do_mapping)
            processor->setFilterFlag(RGBDProcessorFlags::ComputeMapping, true);
        return processor;
    }

}
