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


#include "feature.h"

#include <ntk/image/sift_gpu.h>
#include <ntk/image/sift.h>
#include <ntk/utils/opencv_utils.h>
#include <ntk/numeric/utils.h>
#include <ntk/utils/time.h>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/flann/flann.hpp>

#ifdef HAVE_OPENCV_GREATER_THAN_2_4_0
#include <opencv2/nonfree/nonfree.hpp>
#endif

#include <cassert>

#ifdef HAVE_OPENCV_GREATER_THAN_2_4_0
#include <opencv2/nonfree/nonfree.hpp>
#endif

using namespace cv;

namespace ntk
{

struct FeatureSet :: Impl
{
#ifdef HAVE_OPENCV_GREATER_THAN_2_3_0
    typedef cv::flann::GenericIndex<cv::flann::L2<float> > IndexType;
#else
    typedef cv::flann::Index_<float> IndexType;
#endif
    ntk::Ptr< IndexType > descriptor_index;
};

FeatureSet::FeatureSet()
    : impl(new Impl())
{
}

FeatureSet::~FeatureSet()
{
    delete impl;
}

void FeatureSet :: extractFromImage(const RGBDImage& image,
                                    const FeatureSetParams& params)
{
    ntk::TimeCount tc("FeatureSet::extractFromImage", 1);

    m_extraction_params = params;

    if(!impl->descriptor_index.empty())
        buildDescriptorIndex();

    impl->descriptor_index.release();

    if (params.detector_type == "GPUSIFT")
    {
        ntk_ensure(params.descriptor_type == "SIFT",
                   "Only SIFT descriptor are supported with GPUSIFT detector");
        if (getSiftGPUInstance())
            return extractFromImageUsingSiftGPU(image, params);
        ntk_dbg(0) << "[WARNING] SIFT Gpu cannot be used";
    }
    else if (params.detector_type == "SIFTPP")
    {
        ntk_ensure(params.descriptor_type == "SIFT",
                   "Only SIFT descriptor are supported with SIFTPP detector");
        return extractFromImageUsingSiftPP(image, params);
    }

    cv::FeatureDetector* detector = 0;
    if (params.detector_type == "FAST")
    {
        cv::FeatureDetector* fast_detector
                = new FastFeatureDetector(params.threshold > 0 ? params.threshold : 10,
                                          true/*nonmax_suppression*/);
        detector = new PyramidAdaptedFeatureDetector(fast_detector, 2);
    }
    else if (params.detector_type == "SIFT" || params.detector_type == "GPUSIFT")
    {
#ifdef HAVE_OPENCV_GREATER_THAN_2_4_0
        detector = new SiftFeatureDetector();
#else
        detector = new SiftFeatureDetector(SIFT::DetectorParams::GET_DEFAULT_THRESHOLD(),
                                           SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD());
#endif
    }
    else if (params.detector_type == "SURF")
    {
        detector = new cv::SurfFeatureDetector(params.threshold > 0 ? params.threshold : 400 /*hessian_threshold*/,
                                               3/*octaves*/, 4/*octave_layers*/);
    }
    else if (params.detector_type == "SURF_BIGSCALE")
    {
#ifdef HAVE_OPENCV_GREATER_THAN_2_4_0
#else
        detector = new SurfFeatureDetector(params.threshold > 0 ? params.threshold : 400 /*hessian_threshold*/,
                                           2/*octaves*/, 3/*octave_layers*/);
#endif
    }
    else
    {
        fatal_error(("Point detector not supported: " + params.detector_type).c_str());
    }

    std::vector<cv::KeyPoint> keypoints;
    detector->detect(image.rgbAsGray(), keypoints);
    tc.elapsedMsecs(" -- keypoint extraction -- ");

    cv::DescriptorExtractor* extractor = 0;
    if (params.descriptor_type == "BRIEF32")
    {
        m_feature_type = Feature_BRIEF32;
        extractor = new cv::BriefDescriptorExtractor(32);
    }
    else if (params.descriptor_type == "BRIEF64")
    {
        m_feature_type = Feature_BRIEF64;
        extractor = new cv::BriefDescriptorExtractor(64);
    }
    else if (params.descriptor_type == "SIFT")
    {
        m_feature_type = Feature_SIFT;
        extractor = new cv::SiftDescriptorExtractor();
    }
    else if (params.descriptor_type == "SURF64")
    {
        m_feature_type = Feature_SURF64;
#ifdef HAVE_OPENCV_GREATER_THAN_2_4_0
        extractor = new cv::SurfDescriptorExtractor(400,
                                                    4 /* octaves */,
                                                    2 /* octave layers */,
                                                    false /* extended */);
#else
        extractor = new cv::SurfDescriptorExtractor(400,
                                                    4 /* octaves */,
                                                    2 /* octave layers */,
                                                    false /* extended */);
#endif
    }
    else if (params.descriptor_type == "SURF128")
    {
        m_feature_type = Feature_SURF128;
#ifdef HAVE_OPENCV_GREATER_THAN_2_4_0
        extractor = new cv::SurfDescriptorExtractor(400,
                                                    4 /* octaves */,
                                                    2 /* octave layers */,
                                                    true /* extended */);
#else
        extractor = new cv::SurfDescriptorExtractor(400,
                                                    4 /* octaves */,
                                                    2 /* octave layers */,
                                                    true /* extended */);
#endif
    }
    else
    {
        fatal_error(("Point extractor not supported: " + params.descriptor_type).c_str());
    }

    std::vector<cv::KeyPoint> filtered_keypoints;
    filtered_keypoints.reserve(keypoints.size());

    // Remove keypoints without depth if the option is set.
    if (params.only_features_with_depth)
    {
        const Pose3D depth_pose = image.sensorDepthPose();
        const Pose3D rgb_pose = image.sensorRgbPose();
        foreach_idx(i, keypoints)
        {
            float y = keypoints[i].pt.y;
            float x = keypoints[i].pt.x;
            if (image.rgbPixelHasDepth(y, x))
            {
                float d = image.mappedDepth()(y, x);
                if (image.depthMask().data)
                {
                    cv::Point3f p = rgb_pose.unprojectFromImage(cv::Point3f(x, y, d));
                    p = depth_pose.projectToImage(p);
                    if (is_yx_in_range(image.depthMask(), p.y, p.x) && !image.depthMask()(p.y, p.x))
                        continue;
                }
                filtered_keypoints.push_back(keypoints[i]);
            }
        }
    }
    else
    {
        filtered_keypoints = keypoints;
    }
    ntk_dbg_print(filtered_keypoints.size(), 1);
    ntk_dbg_print(keypoints.size(), 1);
    tc.elapsedMsecs(" -- filtering -- ");

    cv::Mat descriptors;
    extractor->compute(image.rgbAsGray(), filtered_keypoints, descriptors);
    tc.elapsedMsecs(" -- description computation -- ");
    m_descriptor_size = extractor->descriptorSize();
    switch (extractor->descriptorType())
    {
    case CV_32FC1:
        descriptors.copyTo(m_descriptors);
        break;
    case CV_8UC1:
        m_descriptors.create(descriptors.size());
        for_all_rc(descriptors)
                m_descriptors(r,c) = descriptors.at<uchar>(r,c) / 255.0;
        break;
    default:
        ntk_assert(0, "Descriptor type not supported!");
    }

    m_locations.resize(filtered_keypoints.size());
    foreach_idx(i, filtered_keypoints)
            ((KeyPoint&)m_locations[i]) = filtered_keypoints[i];
    fillDepthData(image);
    tc.elapsedMsecs(" -- finishing -- ");
}

void FeatureSet :: extractFromImageUsingSiftGPU(const RGBDImage& image,
                                                const FeatureSetParams& params)
{
    GPUSiftDetector detector;
    m_descriptor_size = 128;

    std::vector<float> descriptors;
    std::vector<KeyPoint> keypoints;

    detector(image.rgbAsGray(), Mat(), keypoints, descriptors);

    m_locations.clear();

    std::vector<bool> enabled_keypoints(keypoints.size(), true);
    if (params.only_features_with_depth)
    {
        m_locations.reserve(keypoints.size());
        foreach_idx(i, keypoints)
        {
            if (image.rgbPixelHasDepth(keypoints[i].pt.y, keypoints[i].pt.x))
            {
                FeaturePoint loc;
                (KeyPoint&)loc = keypoints[i];
                m_locations.push_back(loc);
            }
            else
            {
                enabled_keypoints[i] = false;
            }
        }
    }
    fillDepthData(image);

    m_descriptors = cv::Mat1f(m_locations.size(), 128);
    int enabled_index = 0;
    for (int r = 0; r < descriptors.size()/128; ++r)
    {
        if (!enabled_keypoints[r])
            continue;
        std::copy(&descriptors[r*128],
                  &descriptors[r*128+128],
                  m_descriptors.ptr<float>(enabled_index));
        ++enabled_index;
    }

    m_feature_type = Feature_SIFT;
}

void FeatureSet :: extractFromImageUsingSiftPP(const RGBDImage& image,
                                               const FeatureSetParams& params)
{
    const int levels = 3;
    int O = -1;
    const int S = levels;
    const int omin = -1;
    float const sigman = .5f ;
    float const sigma0 = 1.6f * powf(2.0f, 1.0f / S) ;
    float threshold = 0.01f; // closer to Lowe.
    float edgeThreshold  = 10.0f;
    int unnormalized = 0;
    float magnif = 3.0f;

    cv::Mat1f fim(image.rgbAsGray().size());
    for_all_rc(fim) fim(r, c) = image.rgbAsGray()(r, c) / 255.0;

    if(O < 1)
    {
        O = ntk::math::max
                (int
                 (std::floor
                  (ntk::math::log2
                   (ntk::math::min(fim.cols,fim.rows))) - omin -3), 1);
    }

    VL::Sift sift(fim[0], fim.cols, fim.rows,
                  sigman, sigma0,
                  O, S,
                  omin, -1, S + 1) ;

    sift.detectKeypoints(threshold, edgeThreshold);
    sift.setNormalizeDescriptor(!unnormalized);
    sift.setMagnification(magnif);

    std::vector< std::vector<float> > descriptors;
    m_locations.clear();

    for (VL::Sift::KeypointsConstIter iter = sift.keypointsBegin();
         iter != sift.keypointsEnd(); ++iter)
    {
        FeaturePoint new_location;

        if (params.only_features_with_depth && !image.rgbPixelHasDepth(iter->y,iter->x))
            continue;

        // detect orientations
        VL::float_t angles [4] ;
        int nangles = sift.computeKeypointOrientations(angles, *iter) ;

        // compute descriptors
        for (int a = 0 ; a < nangles ; ++a)
        {
            new_location.pt.x = iter->x;
            new_location.pt.y = iter->y;
            new_location.size = iter->sigma*16;
            new_location.octave = iter->o;
            new_location.angle = angles[a];

            /* compute descriptor */
            VL::float_t descr_pt [128] ;
            sift.computeKeypointDescriptor(descr_pt, *iter, angles[a]) ;

            std::vector<float> desc_vec(128);
            std::copy(descr_pt, descr_pt+128, desc_vec.begin());

            m_locations.push_back(new_location);
            descriptors.push_back(desc_vec);
        } // next angle
    } // next keypoint

    m_descriptors.create(descriptors.size(), 128);
    for_all_rc(m_descriptors)
    {
        m_descriptors(r,c) = descriptors[r][c];
    }

    fillDepthData(image);
    m_feature_type = Feature_SIFT;
    m_descriptor_size = 128;
}

void FeatureSet :: fillDepthData(const RGBDImage& image)
{
    foreach_idx(i, m_locations)
    {
        FeaturePoint& loc = m_locations[i];
        if (image.rgbPixelHasDepth(loc.pt.y, loc.pt.x))
        {
            loc.has_depth = true;
            loc.depth = image.mappedDepth()(loc.pt.y, loc.pt.x);
        }
    }
}

void FeatureSet :: compute3dLocation(const Pose3D& pose)
{
    foreach_idx(i, m_locations)
    {
        FeaturePoint& loc = m_locations[i];
        if (!loc.has_depth)
            continue;
        loc.p3d = pose.unprojectFromImage(loc.pt, loc.depth);
    }
}

void FeatureSet :: draw(const cv::Mat3b& image, cv::Mat3b& display_image) const
{
    std::vector<KeyPoint> keypoints(m_locations.size());
    foreach_idx(i, keypoints)
            keypoints[i] = (const KeyPoint&)m_locations[i];
    cv::Mat dummy;
    image.copyTo(display_image);
    drawKeypoints(image,
                  keypoints,
                  display_image,
                  Scalar(255,0,0,255),
                  DrawMatchesFlags::DRAW_OVER_OUTIMG|DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
}

void FeatureSet :: drawMatches(const cv::Mat3b& image,
                               const cv::Mat3b& rhs_image,
                               const FeatureSet& rhs_features,
                               const std::vector<cv::DMatch>& matches,
                               cv::Mat3b& display_image) const
{
    std::vector<KeyPoint> keypoints1(m_locations.size());
    foreach_idx(i, keypoints1)
            keypoints1[i] = (const KeyPoint&)m_locations[i];
    std::vector<KeyPoint> keypoints2(rhs_features.m_locations.size());
    foreach_idx(i, keypoints2)
            keypoints2[i] = (const KeyPoint&)rhs_features.m_locations[i];

    cv::drawMatches(rhs_image, keypoints2,
                    image, keypoints1,
                    matches, display_image,
                    Scalar(255,0,0,255), Scalar(255,255,0,255),
                    vector<char>());
}

void FeatureSet :: buildDescriptorIndex()
{
    if (m_locations.size() < 1)
        return;
#ifdef HAVE_OPENCV_GREATER_THAN_2_3_0
    cvflann::KDTreeIndexParams params(4);
#else
    cv::flann::KDTreeIndexParams params(4);
#endif
    impl->descriptor_index = new Impl::IndexType(m_descriptors, params);
}

void FeatureSet :: matchWith(const FeatureSet& rhs,
                             std::vector<cv::DMatch>& matches,
                             float ratio_threshold)
{
    ntk_ensure(featureType() == rhs.featureType(), "Cannot match with different feature type.");

    if (!impl->descriptor_index)
        buildDescriptorIndex();

    // Still no index? There was no features in the image.
    if (!impl->descriptor_index)
        return;

    const cv::Mat1f& rhs_descriptors = rhs.descriptors();
    for (int i = 0; i < rhs_descriptors.rows; ++i)
    {
        std::vector<int> indices(2, -1);
        std::vector<float> dists(2, 0);

        std::vector<float> query(descriptorSize());
        std::copy(rhs_descriptors.ptr<float>(i),
                  rhs_descriptors.ptr<float>(i) + rhs_descriptors.cols,
                  query.begin());
#ifdef HAVE_OPENCV_GREATER_THAN_2_3_0
        cvflann::SearchParams params(64);
#else
        cv::flann::SearchParams params(64);
#endif
        assert(!impl->descriptor_index.empty());
        impl->descriptor_index->knnSearch(query, indices, dists, 2, params);
        if (indices[0] < 0 || indices[1] < 0)
            continue;
        const double dist_ratio = dists[0]/dists[1];
        if (dist_ratio > ratio_threshold) // probably wrong match
            continue;

        DMatch m(i, indices[0], -1, dist_ratio);
        matches.push_back(m);
    }
}


} // ntk
