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

#include "located_feature.h"
#include "object_database.h"

#include <ntk/ntk.h>
#include <ntk/image/sift.h>
#include <ntk/image/sift_gpu.h>
#include <ntk/image/feature.h>
#include <ntk/stats/histogram.h>
#include <ntk/stats/distributions.h>

#ifdef HAVE_OPENCV_GREATER_THAN_2_4_0
#   include <opencv2/nonfree/features2d.hpp>
#endif

#include <fstream>

using namespace ntk;
using namespace cv;

namespace ntk
{

std::string
LocatedFeature :: featureTypeName(FeatureType type)
{
    switch (type)
    {
    case Feature_SIFT: return "sift";
    case Feature_SURF64: return "surf64";
    case Feature_SURF128: return "surf128";
    case Feature_FAST: return "fast";
    case Feature_CVFH: return "cvfh";
    case Feature_VFH: return "vfh";
    };
    ntk_throw_exception("Unknown feature type");
    return "unknown";
}

LocatedFeature::FeatureType LocatedFeature :: featureTypeFromName(const std::string& name)
{
    if (name == "sift")
        return Feature_SIFT;
    if (name == "surf64")
        return Feature_SURF64;
    if (name == "surf128")
        return Feature_SURF128;
    if (name == "fast")
        return Feature_FAST;
    if (name == "cvfh")
        return Feature_CVFH;
    if (name == "vfh")
        return Feature_VFH;
    ntk_throw_exception("Unknown type: " + name);
    return Feature_SIFT;
}

LocatedFeature :: ~LocatedFeature()
{
}

FeatureDescriptor::Type LocatedFeature :: descriptorType() const
{
    return descriptorType((FeatureType) m_feature_type);
}

int LocatedFeature :: descriptorSize() const
{
    return descriptorSize((FeatureType) m_feature_type);
}

FeatureDescriptor::Type LocatedFeature :: descriptorType(FeatureType type)
{
    switch (type)
    {
    case Feature_SIFT: return FeatureDescriptor::ByteDescriptor;
    case Feature_SURF64: return FeatureDescriptor::FloatDescriptor;
    case Feature_SURF128: return FeatureDescriptor::FloatDescriptor;
    case Feature_FAST: return FeatureDescriptor::FloatDescriptor;
    case Feature_VFH: return FeatureDescriptor::FloatDescriptor;
    default: ntk_throw_exception("Unknown feature type");
    };
    return (FeatureDescriptor::Type) -1;
}

int LocatedFeature :: descriptorSize(FeatureType type)
{
    switch (type)
    {
    case Feature_SIFT: return 128;
    case Feature_SURF64: return 64;
    case Feature_SURF128: return 128;
    case Feature_FAST: return 64;
    case Feature_VFH: return 308;
    default: ntk_throw_exception("Unknown feature type");
    };
    return -1;
}

void LocatedFeature ::
fillXmlElement(XMLNode& element) const
{
    setXmlAttribute(element, "id-in-indexer", m_id_in_indexer);
    setXmlAttribute(element, "type", (int)m_feature_type);
    setXmlAttribute(element, "has-shared-descriptor", hasSharedDescriptor());
    setXmlAttribute(element, "descriptor-size", descriptorSize());
    if (!hasSharedDescriptor())
        addXmlChild(element, "descriptor", *m_descriptor);
    addXmlChild(element, "location", m_location);
}

void LocatedFeature ::
loadFromXmlElement(const XMLNode& element)
{
    loadFromXmlAttribute(element, "id-in-indexer", m_id_in_indexer);
    {
        int i = 0;
        loadFromXmlAttribute(element, "type", i);
        m_feature_type = i;
    }
    bool has_shared_descriptor = false;
    loadFromXmlAttribute(element, "has-shared-descriptor", has_shared_descriptor);
    int descriptor_size = 128;
    loadFromXmlAttribute(element, "descriptor-size", descriptor_size);
    if (!has_shared_descriptor)
    {
        FeatureDescriptor* descriptor = new FeatureDescriptor(descriptorType(), descriptor_size);
        loadFromXmlChild(element, "descriptor", *descriptor);
        m_descriptor = toPtr(descriptor);
    }

    loadFromXmlChild(element, "location", m_location);
}

void compute_sift_points(std::list<LocatedFeature*>& output, const char* image_file)
{
    ntk_assert(0, "deprecated.");
#if 0
    QString image_fname = QFileInfo(image_file).absoluteFilePath();
    if (QFileInfo(image_fname + ".key").isFile())
    {
        ntk_dbg(1) << image_fname << ".key found, loading it.";
        return load_lowe_keyfile(output, (image_fname + ".key").toUtf8());
    }

    ntk_dbg(1) << "Running Lowe sift program ...";
    QStringList command;
    command << "sift" << " < " << image_fname << " > " << image_fname + ".key";
    ntk_dbg(1) << command.join(" ");
    int exit_code = shell_command(command.join("").toStdString(), true);
    if (exit_code != 0) // FIXME: does not work on mac ?
    {
        QFile(image_fname + ".key").remove();
        ntk_throw_exception("Could not execute sift program.");
    }
    load_lowe_keyfile(output, (image_fname + ".key").toAscii());
#endif
}

void compute_siftgpu_client_points(std::list<LocatedFeature*>& output, const cv::Mat1b& im)
{
    GPUSiftClient client;

    std::vector<float > descriptors;
    std::vector<cv::KeyPoint> keypoints;

    client(im, cv::Mat(),
           keypoints,
           descriptors);

    int n_features = keypoints.size();

    for (int i = 0; i < n_features; ++i)
    {
        FeatureLocation new_location;
        new_location.p_image.x = keypoints[i].pt.x;
        new_location.p_image.y = keypoints[i].pt.y;
        new_location.scale = keypoints[i].size/16.0;
        new_location.orientation = keypoints[i].angle;

        FeatureDescriptor* new_descriptor = new FeatureDescriptor(FeatureDescriptor::ByteDescriptor, 128);
        std::vector<unsigned char>& desc_vec = new_descriptor->byteDescRef();

        foreach_idx(k, desc_vec)
                desc_vec[k] = (unsigned char) (512*descriptors[i*128+k]);

        LocatedFeature* new_point = new LocatedFeature(new_location, new_descriptor);
        output.push_back(new_point);
    }
}

void compute_siftgpu_points(std::list<LocatedFeature*>& output, const cv::Mat1b& im)
{
    SiftGPU* sift = getSiftGPUInstance();
    ntk_ensure(sift, "Cannot use SiftGPU");

    std::vector<float > descriptors;
    std::vector<SiftGPU::SiftKeypoint> keypoints;

    bool ok = sift->RunSIFT(im);

    int n_features = sift->GetFeatureNum();
    keypoints.resize(n_features);
    descriptors.resize(128*n_features);
    sift->GetFeatureVector(&keypoints[0], &descriptors[0]);

    for (int i = 0; i < n_features; ++i)
    {
        FeatureLocation new_location;
        new_location.p_image.x = keypoints[i].x;
        new_location.p_image.y = keypoints[i].y;
        new_location.scale = keypoints[i].s;
        new_location.orientation = keypoints[i].o;

        FeatureDescriptor* new_descriptor = new FeatureDescriptor(FeatureDescriptor::ByteDescriptor, 128);
        std::vector<unsigned char>& desc_vec = new_descriptor->byteDescRef();

        foreach_idx(k, desc_vec)
                desc_vec[k] = (unsigned char) (512*descriptors[i*128+k]);

        LocatedFeature* new_point = new LocatedFeature(new_location, new_descriptor);
        output.push_back(new_point);
    }
}

void compute_siftpp_points(std::list<LocatedFeature*>& output,
                           const cv::Mat1b& im,
                           const cv::Mat1f& depth_image)
{
    const int levels = 3;
    int O = -1;
    const int S = levels;
    const int omin = -1;
    // const int omin = 0; // FIXME: temp ?
    float const sigman = .5 ;
    float const sigma0 = 1.6 * powf(2.0f, 1.0f / S) ;
    // float threshold = 0.04f / levels / 2.0f ;
    float threshold = 0.01; // closer to Lowe.
    float edgeThreshold  = 10.0f;
    int unnormalized = 0;
    float magnif = 3.0;

    VL::PgmBuffer buffer;
    cv::Mat1f fim(im.size());
    for_all_rc(fim) fim(r, c) = im(r, c) / 255.0;

    if(O < 1)
    {
        O = std::max
                (int
                 (std::floor
                  (math::log2
                   (std::min(fim.cols,fim.rows))) - omin -3), 1);
    }

    VL::Sift sift(fim[0], fim.cols, fim.rows,
                  sigman, sigma0,
                  O, S,
                  omin, -1, S + 1) ;

    sift.detectKeypoints(threshold, edgeThreshold);
    sift.setNormalizeDescriptor(!unnormalized);
    sift.setMagnification(magnif);

    for (VL::Sift::KeypointsConstIter iter = sift.keypointsBegin();
         iter != sift.keypointsEnd(); ++iter)
    {
        FeatureLocation new_location;

        // detect orientations
        VL::float_t angles [4] ;
        int nangles = sift.computeKeypointOrientations(angles, *iter) ;

        // compute descriptors
        for (int a = 0 ; a < nangles ; ++a)
        {
            new_location.p_image.x = iter->x;
            new_location.p_image.y = iter->y;
            new_location.scale = iter->sigma;
            new_location.orientation = angles[a];

            /* compute descriptor */
            VL::float_t descr_pt [128] ;
            sift.computeKeypointDescriptor(descr_pt, *iter, angles[a]) ;

            FeatureDescriptor* new_descriptor = new FeatureDescriptor(FeatureDescriptor::ByteDescriptor, 128);
            std::vector<unsigned char>& desc_vec = new_descriptor->byteDescRef();

            foreach_idx(i, desc_vec)
                    desc_vec[i] = (unsigned char) (512*descr_pt[i]);

            LocatedFeature* new_point = new LocatedFeature(new_location, new_descriptor);
            output.push_back(new_point);
        } // next angle
    } // next keypoint
}

void compute_surf_points(std::list<LocatedFeature*>& output, const cv::Mat1b& cv_image, bool extended)
{
    std::vector<cv::KeyPoint> keypoints;
    cv::SURF surf;
    surf.hessianThreshold = 500;
    surf.extended = extended;
    std::vector<float> surf_descriptors;
    surf(cv_image, cv::Mat(), keypoints, surf_descriptors, false);

    FeatureLocation location;
    foreach_idx(key_i, keypoints)
    {
        FeatureDescriptor* new_descriptor = new FeatureDescriptor(FeatureDescriptor::FloatDescriptor, surf.descriptorSize());
        std::vector<float>& descriptor = new_descriptor->floatDescRef();

        const cv::KeyPoint& p = keypoints[key_i];
        location.p_image.x = p.pt.x;
        location.p_image.y = p.pt.y;
        const float haar_min_size = 9;
        location.scale = 1.2*(p.size/haar_min_size);
        // to be compatible with siftpp.
        location.orientation = -ntk::deg_to_rad(p.angle);
        location.laplacian = p.class_id;
        for(int k = 0; k < surf.descriptorSize(); ++k)
        {
            float fv = surf_descriptors[key_i*surf.descriptorSize()+k];
            descriptor[k] = fv;
        }

        LocatedFeature* new_point = new LocatedFeature(location, new_descriptor);
        output.push_back(new_point);
    }
}

void compute_fast_points(std::list<LocatedFeature*>& output, const RGBDImage& image)
{
    FeatureSetParams feature_params("FAST", "BRIEF64", false);
    feature_params.threshold = 20;
    FeatureSet features;
    {
        TimeCount tc_extract("Extract keypoints");
        features.extractFromImage(image, feature_params);
        tc_extract.stop();
    }
    const std::vector<FeaturePoint>& locations = features.locations();
    const cv::Mat1f& descriptors = features.descriptors();

    int n_features = locations.size();

#if 0
    cv::Mat3b debug_img;
    features.draw(image.rgb(), debug_img);
    imwrite("/tmp/debug_fast.png", debug_img);
#endif

    for (int i = 0; i < n_features; ++i)
    {
        FeatureLocation new_location;
        new_location.p_image.x = locations[i].pt.x;
        new_location.p_image.y = locations[i].pt.y;
        new_location.scale = locations[i].size/16.0;
        new_location.orientation = locations[i].angle;

        FeatureDescriptor* new_descriptor = new FeatureDescriptor(FeatureDescriptor::FloatDescriptor, descriptors.cols);
        std::vector<float>& desc_vec = new_descriptor->floatDescRef();

        foreach_idx(k, desc_vec)
        {
            desc_vec[k] = descriptors(i, k);
        }

        LocatedFeature* new_point = new LocatedFeature(new_location, new_descriptor,
                                                       LocatedFeature::Feature_FAST);
        output.push_back(new_point);
    }
}


static void load_cache_file(std::list<LocatedFeature*>& output,
                            const char* filename)
{
    XMLNode node = XMLNode::parseFile(filename, "root");
    for (int i = 0; i < node.nChildNode(); ++i)
    {
        LocatedFeature* p = new LocatedFeature();
        p->loadFromXmlElement(node.getChildNode(i));
        output.push_back(p);
    }
    ntk_dbg(1) << "feature list loaded from cache.";
}

static void write_cache_file(const std::list<LocatedFeature*>& input,
                             const char* filename)
{
    XMLNode node = XMLNode::createXMLTopNode("root");
    foreach_const_it(it, input, std::list<LocatedFeature*>)
    {
        XMLNode child = node.addChild("point");
        (*it)->fillXmlElement(child);
    }
    node.writeToFile(filename);
}

void compute_feature_points(std::list<LocatedFeature*>& output,
                            const char* filename,
                            LocatedFeature::FeatureType type,
                            const char* mask_filename,
                            const cv::Mat1f& depth_image)
{
    QFileInfo cache_file (QString(filename) + "." + LocatedFeature::featureTypeName(type).c_str() + ".key");

    if (cache_file.exists())
    {
        load_cache_file(output, cache_file.absoluteFilePath().toUtf8());
        return;
    }

    cv::Mat3b color_image = imread(filename, 1);
    cv::Mat1b image;
    cvtColor(color_image, image, CV_BGR2GRAY);
    ntk_throw_exception_if(!image.data, "Could not load image.");
    if (mask_filename)
        apply_mask(image, imread(mask_filename, 0));
    compute_feature_points(output, image, type, depth_image);
    write_cache_file(output, cache_file.absoluteFilePath().toUtf8());
}

void compute_feature_points(std::list<LocatedFeature*>& output,
                            const ntk::RGBDImage& im,
                            LocatedFeature::FeatureType type)
{
    QFileInfo cache_file (QString(im.directory().c_str()) + "/keypoints." + LocatedFeature::featureTypeName(type).c_str() + ".key");

    // FIXME: temp, disable cache.
    const bool enable_cache = false;

    if (enable_cache && im.hasDirectory())
    {
        if (cache_file.exists())
        {
            load_cache_file(output, cache_file.absoluteFilePath().toUtf8());
            return;
        }
    }

    ntk::TimeCount tc_fp(" 2 compute_feature_points: ", 1);

    if (type == LocatedFeature::Feature_FAST)
        compute_fast_points(output, im);
    else
        compute_feature_points(output, im.rgbAsGray(), type, im.depth());

    tc_fp.elapsedMsecs(" -- compute itself: ");

    if (im.mappedDepth().data)
    {
        foreach_it(it, output, std::list<LocatedFeature*>)
        {
            Point2f p = (*it)->imagePos();
            double depth = im.mappedDepth()(p.y,p.x);
            if (depth > 1e-3)
            {
                (*it)->locationRef().setDepth(depth);
            }
        }
    }

    tc_fp.elapsedMsecs("-- computing depth: ");

    if (enable_cache && im.hasDirectory())
    {
        write_cache_file(output, cache_file.absoluteFilePath().toUtf8());
    }
}

void compute_feature_points(std::list<LocatedFeature*>& output,
                            const cv::Mat1b& im,
                            LocatedFeature::FeatureType type,
                            const cv::Mat1f& depth_image)
{
    switch (type)
    {
    case LocatedFeature::Feature_SIFT:
        // compute_siftpp_points(output,im, depth_image);
        // compute_siftgpu_points(output, im);
        compute_siftgpu_client_points(output, im);
        break;
    case LocatedFeature::Feature_SURF64:
        compute_surf_points(output,im,false);
        break;
    case LocatedFeature::Feature_SURF128:
        compute_surf_points(output,im,true);
        break;
    case LocatedFeature::Feature_FAST:
        ntk_throw_exception("Fast is only implemented for RGBDImage.");
        break;
    default:
        ntk_throw_exception("This feature type is not implemented from RGBDImage.");
    };

    foreach_const_it(it, output, std::list<LocatedFeature*>)
            (*it)->setFeatureType(type);
}

void compute_feature_points(std::list<LocatedFeature*>& output,
                            const VisualObjectView& object,
                            LocatedFeature::FeatureType type)
{
    compute_feature_points(output, object.grayImage(), type);
    int i = 0;
    foreach_const_it(it, output, std::list<LocatedFeature*>)
    {
        (*it)->setVisualObjectView(object);
        (*it)->setIdInIndexer(i);
        ++i;
    }
}

double entropy(const LocatedFeature& p)
{
    std::vector<unsigned char>::const_iterator desc, desc_end;
    desc = p.byteDescriptors().begin();
    desc_end = p.byteDescriptors().end();

    double norm = 0;
    while (desc != desc_end)
        norm += *desc++;

    desc = p.byteDescriptors().begin();
    double l = 0;
    while (desc != desc_end)
    {
        double v = std::max((*desc++) / norm, 1e-10);
        l += v * log(v);
    }
    return -l;
}

unsigned chi2_distance(const LocatedFeature& p1, const LocatedFeature& p2,
                       unsigned stop_if_greater_than)
{
    ntk_assert(p1.useByteDescriptors() && p2.useByteDescriptors(), "Not for float descritors.");
    std::vector<unsigned char>::const_iterator desc1, desc2, desc1end;
    desc1 = p1.byteDescriptors().begin();
    desc1end = p1.byteDescriptors().end();
    desc2 = p2.byteDescriptors().begin();

    double dist = 0;
    while (desc1 != desc1end && dist <= stop_if_greater_than)
    {
        int sum = *desc1 + *desc2;
        if (sum != 0)
        {
            int diff = *desc1 - *desc2;
            dist += (diff * diff) / double(sum);
        }
        ++desc1; ++desc2;
    }
    return ntk::math::rnd(dist);
}

float euclidian_distance(const FeatureDescriptor& p1, const FeatureDescriptor& p2,
                         float stop_if_greater_than)
{
    ntk_assert (p1.type() == p2.type(), "Incompatible points");

    double dist = 0;

    if (p1.hasFloatDesc() && p2.hasFloatDesc())
    {
        ntk_assert(p1.floatDesc().size() == p2.floatDesc().size(), "Size mismatch.");
        dist = ntk::euclidian_distance(p1.floatDesc(), p2.floatDesc(), stop_if_greater_than);
    }
    else if (p1.hasByteDesc() && p2.hasByteDesc())
    {
        ntk_assert(p1.byteDesc().size() == p2.byteDesc().size(), "Size mismatch.");
        dist = ntk::euclidian_distance(p1.byteDesc(), p2.byteDesc(), stop_if_greater_than);
    }
    else
    {
        ntk_throw_exception("Incompatible feature points.");
    }

    return dist;
}

unsigned emd_distance(const LocatedFeature& p1, const LocatedFeature& p2,
                      unsigned stop_if_greater_than)
{
    std::vector<unsigned char>::const_iterator desc1, desc2, desc1end;
    desc1 = p1.byteDescriptors().begin();
    desc1end = p1.byteDescriptors().end();
    desc2 = p2.byteDescriptors().begin();
    int n = p1.byteDescriptors().size();
    ntk_assert(n%8==0, "Not a multiple of 8 !!");

    unsigned dist = 0;
    for (int i = 0; i < n; i += 8)
    {
        dist += emd_circular_histogram(desc1+i, desc1+i+8, desc2+i);
    }
    return dist;
}

} // end of avs
