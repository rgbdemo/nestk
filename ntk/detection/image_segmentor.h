 #ifndef NTK_IMAGE_SEGMENTOR_H
#define NTK_IMAGE_SEGMENTOR_H

#include <ntk/camera/rgbd_image.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/utils/opencv_utils.h>
#include <ntk/geometry/plane.h>
#include <ntk/geometry/incremental_pose_estimator_from_markers.h>

namespace ntk
{

class ImageSegmentor
{
public:
    ImageSegmentor() {}
    virtual ~ImageSegmentor() {}

public:
    virtual bool initializeFromFirstImage(const RGBDImage& image, const Pose3D& estimated_pose);
    virtual bool filterImage(RGBDImage& image, const Pose3D& estimated_pose) { return true; }

    /*!
     * Pose normalized to visualize the detected object.
     * Depends on the actual implementation, for example the plane
     * segmentors will produce a pose that looks at the plane
     * along the normal, at one meter.
     */
    const Pose3D& getNormalizedPose() const { return normalized_pose; }

    /*!
     * Return the delta between the initial extrinsics of the camera and
     * the normalized pose. Useful for multiple, precalibrated fixed camera
     * setups.
     */
    const Pose3D& getDeltaNormalizedPose() const { return delta_normalized_pose; }

protected:
    Pose3D normalized_pose;
    Pose3D delta_normalized_pose;
};
ntk_ptr_typedefs(ImageSegmentor)

/*!
 * Extract a bounding box in front of a plane. No object has to touch the plane.
 * The normalized view is looking at the plane; along the normal, at a distance of
 * distanceToPlane meters.
 */
class ImageSegmentorFromBackgroundPlane : public ImageSegmentor
{
public:
    ImageSegmentorFromBackgroundPlane();
    virtual ~ImageSegmentorFromBackgroundPlane() {}

public:
    virtual bool initializeFromFirstImage(const RGBDImage& image, const Pose3D& estimated_pose);
    virtual bool filterImage(RGBDImage& image, const Pose3D& estimated_pose);

    const ntk::Rect3f& boundingBox() const { return bounding_box; }
    void setBoundingBox(const ntk::Rect3f& rect) { bounding_box = rect; }

protected:
    float distance_to_plane;
    ntk::Rect3f bounding_box;
    ntk::Plane table_plane;
};
ntk_ptr_typedefs(ImageSegmentorFromBackgroundPlane)

/*! Extract a bounding box around the dominant cluster lying over a plane. */
class ImageSegmentorFromObjectOnPlane : public ImageSegmentorFromBackgroundPlane
{
public:
    ImageSegmentorFromObjectOnPlane() {}
    virtual ~ImageSegmentorFromObjectOnPlane() {}

public:
    virtual bool initializeFromFirstImage(const RGBDImage& image, const Pose3D& estimated_pose);

protected:
    void showCluster(const RGBDImage& image, const std::vector<cv::Point3f>& cluster);
};

class ImageSegmentorFromMarkers : public ntk::ImageSegmentorFromBackgroundPlane
{
public:
    ImageSegmentorFromMarkers(const ntk::MarkerSetup& marker_setup);

public:
    virtual bool initializeFromFirstImage(const ntk::RGBDImage& image, const ntk::Pose3D& estimated_pose);

private:
    ntk::MarkerSetup marker_setup;
};
ntk_ptr_typedefs(ImageSegmentorFromMarkers)

} // namespace ntk

#endif // NTK_IMAGE_SEGMENTOR_H
