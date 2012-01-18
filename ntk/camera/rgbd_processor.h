/**
 * This file is part of the nestk library.
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
 * Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2010
 */

#ifndef NTK_CAMERA_RGBD_PROCESSOR_H
#define NTK_CAMERA_RGBD_PROCESSOR_H

#include <ntk/core.h>
// #include <opencv/cv.h>
#include <ntk/camera/rgbd_image.h>

namespace ntk
{

class RGBDProcessorFlags
{
public:
    enum Flag {
      NoProcessing = 0x0,
      FixGeometry = 0x1, // transform euclidian distance to depth (z component)
      UndistortImages = 0x2, // apply undistort
      FixBias = 0x4, // fix depth bias using polynomials
      FilterAmplitude = 0x8, // remove low amplitude pixels
      FilterNormals = 0x10, // remove normals not aligned enough with line of sight
      FilterUnstable = 0x20, // remove depth values changing too much between consecutive frames
      FilterEdges = 0x40, // remove pixels with high depth spatial derivative
      FilterThresholdDepth = 0x80, // set a depth range
      FilterMedian = 0x100,
      ComputeMapping = 0x200, // compute mappings between rgb and depth
      ComputeNormals = 0x400, // compute normals (required by FilterNormals)
      ComputeKinectDepthTanh = 0x800, // compute depth in meters from kinect values
      ComputeKinectDepthLinear = 0x1000, // compute depth in meters from kinect values
      ComputeKinectDepthBaseline = 0x2000, // compute depth in meters from kinect values
      NoAmplitudeIntensityUndistort = 0x4000, // apply undistort
      Pause = 0x8000, // disable temporary the processing
      RemoveSmallStructures = 0x10000,
      FillSmallHoles = 0x20000,
      FlipColorImage = 0x40000, // horizontally flip the color image
      NiteProcessed = 0x80000, // raw = mapped = postprocessed
      FilterBilateral = 0x100000
    };

    RGBDProcessorFlags(int flags) { m_flags = flags; }

    /*! Accessors to the flag values */
    bool hasFilterFlag(Flag flag) const { return m_flags&flag; }
    void setFilterFlags(int flags) { m_flags = flags; }
    void setFilterFlag(Flag flag, bool enabled)
    { if (enabled) m_flags |= flag; else m_flags &= ~flag; }

    void addFlags(const RGBDProcessorFlags& rhs)
    {
        m_flags |= rhs.m_flags;
    }

    operator int() { return m_flags; }

private:
    int m_flags;
};

/*!
 * Process raw RGB-D images to generate postprocessed members.
 * Various options are available through flags.
 * @see KinectProcessor
 */
class RGBDProcessor
{
public:


public:
  RGBDProcessor();

public:
  /*! Accessors to the flag values */
  bool hasFilterFlag(RGBDProcessorFlags::Flag flag) const { return m_flags.hasFilterFlag(flag); }
  void setFilterFlags(int flags) { m_flags.setFilterFlags(flags); }
  void setFilterFlag(RGBDProcessorFlags::Flag flag, bool enabled) { m_flags.setFilterFlag(flag, enabled); }
  void setFilterFlags(const RGBDProcessorFlags& flags) { m_flags = flags; }
  RGBDProcessorFlags getFilterFlags() const { return m_flags; }

  /*! Set the depth range. */
  void setMinDepth(float meters) { m_min_depth = meters; }
  float minDepth() const { return m_min_depth; }
  void setMaxDepth(float meters) { m_max_depth = meters; }
  float maxDepth() const { return m_max_depth; }

  /*! Set the amplitude range. */
  void setMinAmplitude(float value) { m_min_amplitude = value; }
  float minAmplitude() const { return m_min_amplitude; }
  void setMaxAmplitude(float value) { m_max_amplitude = value; }
  float maxAmplitude() const { return m_max_amplitude; }

  /*!
   * Set the maximal angle between camera vector and
   * normal vector for normal filter.
   */
  void setMaxNormalAngle(float angle_in_deg) { m_max_normal_angle = angle_in_deg; }

  /*! Parameter of the time stability filter. */
  void setMaxTimeDelta(float v) { m_max_time_depth_delta = v; }

  /*! Max depth difference for the edge filter. */
  void setMaxSpacialDelta(float v) { m_max_spatial_depth_delta = v; }

  /*!
   * Resolution factor for depth/rgb mapping.
   * Especially useful when the depth image has a lower resolution.
   */
  void setMappingResolution(float r) { m_mapping_resolution = r; }

public:
  /*! Postprocess an RGB-D image. This function is not reentrant. */
  virtual void processImage(RGBDImage& image);

  /*! Compute normals using PCL integral images. This function is reentrant. */
  virtual void computeNormalsPCL(RGBDImage& image);

  /*! Compute normals using PCL local plane estimation with PCA. */
  virtual void computeHighQualityNormalsPCL(RGBDImage& image);

  /*! Filter the depth image using a tuned bilateral filter. */
  virtual void bilateralFilter(RGBDImage& image);

protected:
  virtual void undistortImages();
  virtual void fixDepthGeometry();
  virtual void fixDepthBias();
  virtual void removeLowAmplitudeOutliers();
  virtual void removeNormalOutliers();
  virtual void removeUnstableOutliers();
  virtual void removeEdgeOutliers();
  virtual void applyDepthThreshold();
  virtual void computeNormals();
  virtual void computeMappings();
  virtual void medianFilter();
  virtual void computeKinectDepthLinear();
  virtual void computeKinectDepthTanh();
  virtual void computeKinectDepthBaseline();
  virtual void removeSmallStructures();
  virtual void fillSmallHoles();

protected:
  RGBDImage* m_image;
  RGBDProcessorFlags m_flags;
  cv::Mat1f m_last_depth_image;
  float m_min_depth;
  float m_max_depth;
  float m_max_normal_angle;
  float m_max_time_depth_delta;
  float m_max_spatial_depth_delta;
  float m_mapping_resolution;
  float m_min_amplitude;
  float m_max_amplitude;
};
ntk_ptr_typedefs(RGBDProcessor)

/*! RGBDProcessor with default parameters for libfreenect Kinect driver. */
class FreenectRGBDProcessor : public RGBDProcessor
{
public:
  FreenectRGBDProcessor()
    : RGBDProcessor()
  {
    setFilterFlag(RGBDProcessorFlags::ComputeKinectDepthBaseline, true);
    setFilterFlag(RGBDProcessorFlags::NoAmplitudeIntensityUndistort, true);
  }
};

// For backward compatiblity.
typedef FreenectRGBDProcessor KinectProcessor;

/*! RGBDProcessor with default parameters for OpenNI/Nite. */
class OpenniRGBDProcessor : public RGBDProcessor
{
public:
  OpenniRGBDProcessor()
    : RGBDProcessor()
  {
    // Everything is done by the grabber.
    setFilterFlags(RGBDProcessorFlags::NiteProcessed | RGBDProcessorFlags::ComputeMapping);
  }

protected:
  virtual void computeMappings();
};

// For backward compatiblity.
typedef OpenniRGBDProcessor NiteProcessor;

class RGBDProcessorFactory
{
public:
    struct Params
    {
        Params() : camera_type("kinect-ni"), do_mapping(true)
        {}

        std::string camera_type;
        bool do_mapping;
    };

public:
    RGBDProcessor* createProcessor(const RGBDProcessorFactory::Params& params);
};

/*!
 * Compute a false color image from a depth map.
 * Optional min/max values can be provided to force the value range.
 */
void compute_color_encoded_depth(const cv::Mat1f& depth, cv::Mat3b& color_dept,
                                 double* min_val = 0, double* max_val = 0);

} // ntk

#endif // NTK_CAMERA_RGBD_PROCESSOR_H
