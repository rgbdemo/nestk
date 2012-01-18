//
// Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2009
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.


#include "object_database.h"
#include <ntk/ntk.h>
#include <ntk/mesh/mesh_renderer.h>
#include <ntk/camera/rgbd_frame_recorder.h>

#include <memory>

using namespace ntk;
using namespace cv;

namespace ntk
{

static cv::Rect compute_bounding_rect(const cv::Mat1b& input)
{
    cv::Mat1b im = input;
    ntk_assert(im.data != 0, cv::format("Could not read image\n").c_str());
    cv::threshold(im, im, 1, 255, cv::THRESH_BINARY);
    std::vector< std::vector<cv::Point> > contours;
    cv::findContours(im, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    ntk_assert(contours.size() > 0, "Empty view!");
    cv::Rect rect = cv::boundingRect(cv::Mat(contours[0]));
    for (int i = 1; i < contours.size(); ++i)
    {
        rect = rect | cv::boundingRect(cv::Mat(contours[i]));
    }
    return rect;
}

VisualObjectView :: VisualObjectView(const VisualObject* object,
                                     const std::string& image,
                                     const VisualObjectViewId& id)
    : m_object(object),
      m_id(id),
      m_image_dir(image),
      m_image_width(-1),
      m_image_height(-1),
      m_has_depth(0),
      m_has_mask(0),
      m_has_inner_mask(0)
{
    buildRGBDImageCache();

    cv::Mat1b im = grayImage();
    if (!im.data)
        ntk_throw_exception("Trying to construct a VisualObject view with an invalid image file: "
                            + image);

    m_image_width = im.cols;
    m_image_height = im.rows;
    // m_bounding_rect = cv::Rect(0, 0, im.cols, im.rows);
    m_bounding_rect = compute_bounding_rect(im);

    if (is_file(imageDir() + "/rgb_pose.avs"))
    {
        m_object_pose.parseAvsFile((imageDir() + "/rgb_pose.avs").c_str());
        ntk_dbg(1) << "[OK] load pose for view " << name();
    }
    else
    {
        ntk_dbg(1) << "[Warning] could NOT load pose for view " << name();
    }
}

void VisualObjectView :: buildCache()
{
    if (m_object && m_object->has3DModel())
        buildFromModel();
    else
        buildFromPrecomputed();
}

void VisualObjectView :: buildRGBDImageCache()
{
    ntk::TimeCount tc_build_image("buildRGBDImageCache", 1);
    RGBDCalibration calib_data;
    ntk_ensure(is_file(m_image_dir + "/../calibration.yml"), "Calibration data not found");
    calib_data.loadFromFile((m_image_dir + "/../calibration.yml").c_str());
    RGBDProcessorFactory factory;
    RGBDProcessorFactory::Params params;
    params.camera_type = calib_data.camera_type;
    params.do_mapping = true;
    std::auto_ptr<RGBDProcessor> processor (factory.createProcessor(params));
    tc_build_image.elapsedMsecs(" -- preparation: ");

    RGBDImage image (m_image_dir, &calib_data, processor.get());
    tc_build_image.elapsedMsecs(" -- loading image: ");
    medianBlur(image.mappedDepthRef(), image.mappedDepthRef(), 3);
    tc_build_image.elapsedMsecs(" -- median filtering: ");
    imwrite(m_image_dir + "/color.png", image.rgb());
    imwrite_Mat1f_raw(m_image_dir + "/mapped_depth.raw", image.mappedDepth());
#ifdef NESTK_HEAVY_DEBUG
    imwrite_normalized(m_image_dir + "/mapped_depth.png", image.mappedDepth());
#endif
    tc_build_image.elapsedMsecs(" -- writing images: ");
}

void VisualObjectView :: buildFromModel()
{
    TimeCount tc_build("buildFromModel", 1);
    const int max_contour_points = 500;
    const int max_depth_samples = 1000;

    ntk_ensure(m_object && m_object->has3DModel(), "Cannot build without 3D model.");
    ntk_ensure(m_object_pose.isValid(), "3D model but no rgb_pose.avs file found!");

    cv::Mat4b proj_img_4b (m_image_height, m_image_width);
    proj_img_4b = Vec4b(0,0,0,255);

    Mesh mesh;
    m_object->loadMesh(mesh);
    tc_build.elapsedMsecs(" -- loadMesh: ");

    MeshRenderer renderer(m_image_width, m_image_height);
    renderer.setMesh(mesh);
    renderer.setPose(m_object_pose);

    tc_build.elapsedMsecs(" -- prepare rendering: ");
    renderer.renderToImage(proj_img_4b, MeshRenderer::NORMAL);
    tc_build.elapsedMsecs(" -- renderToImage: ");

#ifdef NESTK_HEAVY_DEBUG
    imwrite("/tmp/debug_projected.png", toMat3b(proj_img_4b));
#endif

#ifdef NESTK_NEED_DEPTH_SAMPLES
    cv::Mat1f depth_im = renderer.depthBuffer();

    // depth samples
    {
        m_depth_samples.clear();

        int nb_depth_points = cv::countNonZero(depth_im);
        int delta = std::max(nb_depth_points / max_depth_samples, 1);

        cv::RNG rng;
        for_all_rc(depth_im)
        {
            float depth = depth_im(r,c);

            if (depth < 1e-5)
                continue;

            if (rng(delta) == 0)
                m_depth_samples.push_back(Point3f(c,r,depth));
        }
        ntk_dbg_print(m_depth_samples.size(), 1);
        cv::Mat1b debug_samples(depth_im.size());
        debug_samples = 0u;
        foreach_idx(i, m_depth_samples)
        {
            cv::Point3f p = m_depth_samples[i];
            debug_samples(p.y, p.x) = 255;
        }
        imwrite("/tmp/debug_samples.png", debug_samples);
    }
    tc_build.elapsedMsecs(" -- get depth samples: ");
#endif

#ifdef NESTK_DISABLED_ADS_NOT_YET_IMPORTED
    // depth contour
    {
        cv::Mat1b depth_edge_image = compute_edge_image_from_depth(depth_im);
        int nb_edge_points = (depth_edge_image.cols * depth_edge_image.rows) - cv::countNonZero(depth_edge_image);
        int delta = std::max(nb_edge_points / max_contour_points, 1);
        m_depth_edge_contour.clear();
        int contour_i = 0;
        cv::RNG rng;
        for_all_rc(depth_edge_image)
        {
            if (depth_edge_image(r,c))
                continue;

            if (rng(delta) == 0)
                m_depth_edge_contour.push_back(Point2f(c,r));

            ++contour_i;
        }
        ntk_dbg_print(m_depth_edge_contour.size(), 1);
        cv::Mat1b debug_edges(depth_edge_image.size());
        debug_edges = 0u;
        foreach_idx(i, m_depth_edge_contour)
        {
            cv::Point2f p = m_depth_edge_contour[i];
            debug_edges(p.y, p.x) = 255;
        }
        imwrite("/tmp/debug_depth_contour.png", debug_edges);
    }
#endif

    cv::Mat1b proj_img_1b; cvtColor(toMat3b(proj_img_4b), proj_img_1b, CV_RGB2GRAY);
    cv::Mat1b inner_mask;
    cv::threshold(proj_img_1b, inner_mask, 1, 255, cv::THRESH_BINARY);
    imwrite(innerMaskImageFile().c_str(), inner_mask);
    m_has_inner_mask = true;

    cv::Rect bbox = compute_bounding_rect(inner_mask);
    float delta_x = bbox.width*0.2;
    float delta_y = bbox.height*0.2;
    bbox.width += delta_x;
    bbox.x -= delta_x/2.0;
    bbox.height += delta_y;
    bbox.y -= delta_y/2.0;
    adjustRectToImage(bbox, inner_mask.size());

    inner_mask(bbox) = 255.0;
    imwrite(maskImageFile().c_str(), inner_mask);
    m_has_mask = true;

    imwrite_Mat1f_raw(depthImageFile(), renderer.depthBuffer());
    imwrite_normalized(m_image_dir + "/mapped_depth.png", renderer.depthBuffer());
    if (QFileInfo(depthImageFile().c_str()).isFile())
        m_has_depth = true;
    tc_build.elapsedMsecs(" -- write image: ");
}

void VisualObjectView :: buildFromPrecomputed()
{
    if (QFileInfo(maskImageFile().c_str()).isFile())
        m_has_mask = true;

    if (QFileInfo(innerMaskImageFile().c_str()).isFile())
        m_has_inner_mask = true;

    if (QFileInfo(depthImageFile().c_str()).isFile())
        m_has_depth = true;
}

std::string VisualObjectView :: name() const
{
    return cv::format("%s-%d", object().name().c_str(), m_id.view_id_in_object);
}

cv::Mat1b VisualObjectView :: maskImage() const
{
    std::string mask_file = maskImageFile();
    if (!QFileInfo(mask_file.c_str()).isFile())
        return cv::Mat1b();
    return imread(mask_file, 0);
}

cv::Mat1b VisualObjectView :: innerMaskImage() const
{
    std::string mask_file = innerMaskImageFile();
    if (!QFileInfo(mask_file.c_str()).isFile())
        return cv::Mat1b();
    return imread(mask_file, 0);
}

cv::Mat1b VisualObjectView :: grayImage() const
{
    cv::Mat1b gray_im;
    cvtColor(colorImage(), gray_im, CV_BGR2GRAY);
    if (hasMask())
        apply_mask(gray_im, maskImage());
    return gray_im;
}

cv::Mat1f VisualObjectView :: mappedDepthImage() const
{
    cv::Mat1f im = imread_Mat1f_raw(depthImageFile());
    return im;
}

cv::Mat3b VisualObjectView :: colorImage() const
{
    cv::Mat3b im = imread(colorImageFile(), 1);
    if (hasMask())
        apply_mask(im, maskImage());
    return im;
}

void VisualObjectView :: fillXmlElement(XMLNode& element) const
{
    setXmlAttribute(element, "image_name", QFileInfo(m_image_dir.c_str()).fileName());
    setXmlAttribute(element, "has_depth", m_has_depth);
    setXmlAttribute(element, "has_mask", m_has_mask);
    setXmlAttribute(element, "has_inner_mask", m_has_inner_mask);
    addXmlChild(element, "object_pose", m_object_pose);
    addXmlRawTextDataChild(element, "bounding_rect", m_bounding_rect);
    setXmlAttribute(element, "image_width", m_image_width);
    setXmlAttribute(element, "image_height", m_image_height);
    addXmlRawTextDataChild(element, "depth_contour", m_depth_edge_contour);
    addXmlRawTextDataChild(element, "depth_samples", m_depth_samples);
}

void VisualObjectView :: loadFromXmlElement(const XMLNode& element, XmlContext* context)
{
    const VisualObject* object = dynamic_cast<const VisualObject*>(context);
    ntk_throw_exception_if(!object, "Missing object context.");

    QString image_name;
    loadFromXmlAttribute(element, "image_name", image_name);
    m_image_dir = object->directory() + "/" + image_name.toStdString();
    ntk_throw_exception_if(!QFileInfo(m_image_dir.c_str()).isDir(), "Directory " + m_image_dir + " does not exist.");

    loadFromXmlAttribute(element, "has_depth", m_has_depth);
    loadFromXmlAttribute(element, "has_mask", m_has_mask);
    loadFromXmlAttribute(element, "has_inner_mask", m_has_inner_mask);

    ntk_throw_exception_if(m_has_depth && !QFileInfo(depthImageFile().c_str()).isFile(),
                           "File " + depthImageFile() + " does not exist.");

    ntk_throw_exception_if(m_has_mask && !QFileInfo(maskImageFile().c_str()).isFile(),
                           "File " + maskImageFile() + " does not exist.");

    ntk_throw_exception_if(m_has_inner_mask && !QFileInfo(innerMaskImageFile().c_str()).isFile(),
                           "File " + innerMaskImageFile() + " does not exist.");

    loadFromXmlChild(element, "object_pose", m_object_pose);

    loadFromXmlRawTextDataChild(element, "bounding_rect", m_bounding_rect);
    loadFromXmlAttribute(element, "image_width", m_image_width);
    loadFromXmlAttribute(element, "image_height", m_image_height);
    loadFromXmlRawTextDataChild(element, "depth_contour", m_depth_edge_contour);
    loadFromXmlRawTextDataChild(element, "depth_samples", m_depth_samples);
}

void VisualObjectView :: transformPose(const cv::Vec3f& tranlation, const cv::Vec3f& rotation)
{
    m_object_pose.applyTransformBefore(tranlation, rotation);
}

}
