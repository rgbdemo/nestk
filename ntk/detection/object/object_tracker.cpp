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

#include "object_tracker.h"

using namespace ntk;
using namespace cv;

namespace ntk
{

  ObjectTracker::ObjectTracker(const ObjectMatch& match)
    : m_nframes_without_detection(0),
    m_nframes_with_detection(0),
    m_has_updated_pose(false)
  {
    m_last_pose = match.pose()->pose3d();
    m_raw_pose = match.pose()->pose3d();
    m_estimated_pose = m_last_pose;
    m_model = &(match.model());
    m_last_projected_bounding_rect = bounding_box(match.pose()->projectedBoundingRect());

    /*
     * 12 variables
     * tx, Dtx, ty, Dy, tz, Dtz, rx, Drx, ry, Dry, rz, Drz
     * with D the derivative.
     * This tracking assumes a constant speed model.
     * Rotation are given as axis / angle as magnitude
     * representation.
     */
    m_kalman = KalmanFilter(12, 12, 0);
    Mat1f state = Mat(12, 1, CV_32FC1);

    Mat1f process_noise (12, 1);
    Mat1f measurement (12, 1);
    Mat1f measurement_noise (12, 1);

    measurement = 0.f;

    /*
     * If there were only two variables, this generates a matrix
     * with the following form:
     * 1 1 0 0
     * 0 1 0 0
     * 0 0 1 1
     * 0 0 1 0
     */
    m_kalman.transitionMatrix = Mat1f(12, 12);
    setIdentity(m_kalman.transitionMatrix);
    for (int i = 0; i < 12; i += 2)
    {
      m_kalman.transitionMatrix.at<float>(i,i+1) = 1;
    }

    setIdentity(m_kalman.measurementMatrix, 1);
    setIdentity(m_kalman.processNoiseCov, 1e-5);
    setIdentity(m_kalman.measurementNoiseCov, 1e-1);
    setIdentity(m_kalman.errorCovPost, 1);

    cv::Vec3f r = m_last_pose.cvRodriguesRotation();
    cv::Vec3f t = m_last_pose.cvTranslation();
    // derivatives are null initially.
    ((Mat1f)m_kalman.statePost) << t[0], 0, t[1], 0, t[2], 0, r[0], 0, r[1], 0, r[2], 0;
  }

  void ObjectTracker :: prepareForNewFrame()
  {
    m_last_pose = m_estimated_pose;
    ++m_nframes_without_detection;
    m_has_updated_pose = false;
    cv::Mat1f pred = m_kalman.predict();
    m_estimated_pose.resetCameraTransform();
    m_estimated_pose.applyTransformBeforeRodrigues(Vec3f(pred(0,0), pred(2,0), pred(4,0)),
                                                   Vec3f(pred(6,0), pred(8,0), pred(10,0)));
  }

  double ObjectTracker :: compatibilityMeasure(const ObjectMatch& match) const
  {
    if (&(match.model()) != m_model) return -1;

    cv::Rect r1 = m_last_projected_bounding_rect;
    cv::Rect r2 = bounding_box(match.projectedBoundingRect());

    double overlap = ntk::overlap_ratio(r1,r2);
    return overlap;
  }

  void ObjectTracker :: addNewDetection(const ObjectMatch& match)
  {
    m_raw_pose = match.pose()->pose3d();

    cv::Vec3f r = match.pose()->pose3d().cvRodriguesRotation();
    cv::Vec3f t = match.pose()->pose3d().cvTranslation();

    cv::Vec3f dr = r - m_estimated_pose.cvRodriguesRotation();
    cv::Vec3f dt = t - m_estimated_pose.cvTranslation();

    cv::Mat1f measurement(12,1);
    measurement << t[0], dt[0], t[1], dt[1], t[2], dt[2],
                   r[0], dr[0], r[1], dr[1], r[2], dr[2];

    Mat1f new_state = m_kalman.correct(measurement);
    m_estimated_pose.resetCameraTransform();
    m_estimated_pose.applyTransformBeforeRodrigues(Vec3f(new_state(0,0), new_state(2,0), new_state(4,0)),
                                                   Vec3f(new_state(6,0), new_state(8,0), new_state(10,0)));
    ntk_dbg_print(m_last_pose, 1);
    ntk_dbg_print(m_estimated_pose , 1);
    ntk_dbg_print(m_raw_pose , 1);
    m_last_projected_bounding_rect = bounding_box(match.projectedBoundingRect());
    m_nframes_without_detection = 0;
    ++m_nframes_with_detection;
    m_has_updated_pose = true;
  }

} // avs
