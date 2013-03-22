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

#ifndef NTK_GEOMETRY_INCREMENTAL_POSE_ESTIMATOR_H
#define NTK_GEOMETRY_INCREMENTAL_POSE_ESTIMATOR_H

#include <ntk/geometry/pose_3d.h>
#include <ntk/camera/rgbd_image.h>
#include <ntk/camera/rgbd_calibration.h>
#include <ntk/image/feature.h>

namespace ntk
{

class IncrementalPoseEstimator
{
public:
    virtual ~IncrementalPoseEstimator () {}

    virtual bool estimateCurrentPose() = 0;
    virtual void reset() { m_current_pose = Pose3D(); }
    virtual IncrementalPoseEstimator* clone() const = 0;

public:
    //! Return the last estimated pose.
    const Pose3D& currentPose() const { return m_current_pose; }
    Pose3D& currentPose() { return m_current_pose; }
    virtual void setCurrentPose(const std::string& camera_serial, const Pose3D& pose) { m_current_pose = pose; }

protected:
    Pose3D m_current_pose;
};
ntk_ptr_typedefs(IncrementalPoseEstimator)

class IncrementalPoseEstimatorFromImage : public IncrementalPoseEstimator
{
public:
    struct FeatureCorrespondance
    {
        int feature_index;
        int train_feature_index;
        int train_image_index;
        float distance;
    };

public:
    virtual void addNewImage(const RGBDImage& image)
    { image.copyTo(m_new_image); }

    virtual ntk::Ptr<FeatureSet> getLastImageFeatures() const { return ntk::Ptr<FeatureSet>(); }

    virtual IncrementalPoseEstimatorFromImage* clone() const = 0;

protected:
    RGBDImage m_new_image;
};
ntk_ptr_typedefs(IncrementalPoseEstimatorFromImage)

/*!
 * Compute relative pose information using viewXXXX/relative_pose.avs file.
 * The image should have directory information (i.e. loaded from disk)
 * and have a "relative_pose.avs" file storing the pose information.
 * This file should be readableby the Pose3D::parseAvsFile() function.
 */
class IncrementalPoseEstimatorFromFile : public IncrementalPoseEstimatorFromImage
{
public:
    virtual bool estimateCurrentPose();
    virtual IncrementalPoseEstimatorFromFile* clone() const { return new IncrementalPoseEstimatorFromFile(*this); }
};

/*!
 * Compute relative pose information by applying a constant delta pose.
 * This pose estimator takes an initial pose and a delta pose, and for
 * each new frame the current_pose gets multiplied by the delta_pose.
 */
class IncrementalPoseEstimatorFromDelta : public IncrementalPoseEstimatorFromImage
{
public:
    IncrementalPoseEstimatorFromDelta(const Pose3D& initial_pose,
                                      const Pose3D& delta_pose)
        : m_initial_pose(initial_pose),
          m_delta_pose(delta_pose)
    {
        reset();
    }

    virtual bool estimateCurrentPose();

    virtual void reset() { m_current_pose = m_initial_pose; }

    virtual IncrementalPoseEstimatorFromDelta* clone() const { return new IncrementalPoseEstimatorFromDelta(*this); }

private:
    Pose3D m_initial_pose;
    Pose3D m_delta_pose;
};

class DummyIncrementalPoseEstimator : public IncrementalPoseEstimatorFromImage
{
public:
    virtual bool estimateCurrentPose()
    {
        if (!m_current_pose.isValid())
            m_current_pose = *m_new_image.calibration()->depth_pose;
        return true;
    }

    virtual DummyIncrementalPoseEstimator* clone() const { return new DummyIncrementalPoseEstimator(*this); }
};

} // ntk

#endif // NTK_GEOMETRY_INCREMENTAL_POSE_ESTIMATOR_H
