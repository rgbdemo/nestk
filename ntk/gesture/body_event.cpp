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


#include "body_event.h"
#include <ntk/gesture/ni_utils.h>
#include <ntk/utils/time.h>

#include <QWriteLocker>

#include <fstream>

using namespace cv;
using namespace ntk;

namespace ntk
{

const NtkDebug& operator<<(const NtkDebug& os, const BodyEvent& e)
{
    std::string name = "Unknown.";
    switch(e.kind)
    {
    case BodyEvent::CircleEvent:
        os << "[Event CIRCLE] " << "radius=" << e.velocity << " ntimes=" << e.angle;
        break;
    case BodyEvent::PushEvent:
        os << "[Event PUSH] " << "vel=" << e.velocity << " angle=" << e.angle;
        break;
    case BodyEvent::SteadyEvent:
        os << "[Event STEADY] " << "vel=" << e.velocity << " angle=" << e.angle;
        break;
    case BodyEvent::SwipeDownEvent:
        os << "[Event SWIPE DOWN] " << "vel=" << e.velocity << " angle=" << e.angle;
        break;
    case BodyEvent::SwipeUpEvent:
        os << "[Event SWIPE UP] " << "vel=" << e.velocity << " angle=" << e.angle;
        break;
    case BodyEvent::SwipeLeftEvent:
        os << "[Event SWIPE LEFT] " << "vel=" << e.velocity << " angle=" << e.angle;
        break;
    case BodyEvent::SwipeRightEvent:
        os << "[Event SWIPE RIGHT] " << "vel=" << e.velocity << " angle=" << e.angle;
        break;
    case BodyEvent::WaveEvent:
        os << "[Event WAVE]";
        break;
    default:
        break;
    }

    return os;
}

} // ntk

namespace
{

void XN_CALLBACK_TYPE BodyEventDetectorSessionStart(const XnPoint3D& ptFocus, void *pUserCxt)
{
    ntk::BodyEventDetector* detector = reinterpret_cast<ntk::BodyEventDetector*>(pUserCxt);
    detector->sessionStartedCallback();
}

void XN_CALLBACK_TYPE BodyEventDetectorSessionEnd(void *pUserCxt)
{
    ntk::BodyEventDetector* detector = reinterpret_cast<ntk::BodyEventDetector*>(pUserCxt);
    detector->sessionFinishedCallback();
}

// Circle detector
void XN_CALLBACK_TYPE BodyEventDetectorCircle_Circle(XnFloat fTimes,
                                                     XnBool bConfident,
                                                     const XnVCircle* pCircle,
                                                     void* cxt)
{
    ntk::BodyEventDetector* detector = reinterpret_cast<ntk::BodyEventDetector*>(cxt);
    // Ensure there are no multiple detections.
    detector->getCircleDetector()->Reset();
    BodyEvent event(BodyEvent::CircleEvent, pCircle->fRadius, fTimes);
    detector->sendEvent(event);
}



// Push detector
void XN_CALLBACK_TYPE BodyEventDetectorPush_Pushed(XnFloat fVelocity, XnFloat fAngle, void* cxt)
{
    ntk::BodyEventDetector* detector = reinterpret_cast<ntk::BodyEventDetector*>(cxt);
    BodyEvent event(BodyEvent::PushEvent, fVelocity, fAngle);
    detector->sendEvent(event);
}

// Wave detector
void XN_CALLBACK_TYPE BodyEventDetectorWave_Waved(void* cxt)
{
    ntk::BodyEventDetector* detector = reinterpret_cast<ntk::BodyEventDetector*>(cxt);
    BodyEvent event(BodyEvent::WaveEvent, 0, 0);
    detector->sendEvent(event);
}

// Steady detector
void XN_CALLBACK_TYPE BodyEventDetectorSteady_Steady(XnUInt32 id, XnFloat fStdDev, void* cxt)
{
    ntk::BodyEventDetector* detector = reinterpret_cast<ntk::BodyEventDetector*>(cxt);
    BodyEvent event(BodyEvent::SteadyEvent, fStdDev, 0);
    detector->sendEvent(event);
}

// Swipe detector.
static void XN_CALLBACK_TYPE BodyEventDetectorSwipe_SwipeUp(XnFloat fVelocity, XnFloat fAngle, void* cxt)
{
    ntk::BodyEventDetector* detector = reinterpret_cast<ntk::BodyEventDetector*>(cxt);
    BodyEvent event(BodyEvent::SwipeUpEvent, fVelocity, fAngle);
    detector->sendEvent(event);
}

static void XN_CALLBACK_TYPE BodyEventDetectorSwipe_SwipeDown(XnFloat fVelocity, XnFloat fAngle, void* cxt)
{
    ntk::BodyEventDetector* detector = reinterpret_cast<ntk::BodyEventDetector*>(cxt);
    BodyEvent event(BodyEvent::SwipeDownEvent, fVelocity, fAngle);
    detector->sendEvent(event);
}

static void XN_CALLBACK_TYPE BodyEventDetectorSwipe_SwipeLeft(XnFloat fVelocity, XnFloat fAngle, void* cxt)
{
    ntk::BodyEventDetector* detector = reinterpret_cast<ntk::BodyEventDetector*>(cxt);
    BodyEvent event(BodyEvent::SwipeLeftEvent, fVelocity, fAngle);
    detector->sendEvent(event);
}

static void XN_CALLBACK_TYPE BodyEventDetectorSwipe_SwipeRight(XnFloat fVelocity, XnFloat fAngle, void* cxt)
{
    ntk::BodyEventDetector* detector = reinterpret_cast<ntk::BodyEventDetector*>(cxt);
    BodyEvent event(BodyEvent::SwipeRightEvent, fVelocity, fAngle);
    detector->sendEvent(event);
}

}

namespace ntk
{

void BodyEventDetector :: initialize(xn::Context& context, xn::DepthGenerator& depth_generator)
{
    m_context = &context;
    m_depth_generator = &depth_generator;
    m_session_manager = new XnVSessionManager();
    XnStatus rc = m_session_manager->Initialize(m_context, "Click,Wave", "RaiseHand");
    if (rc != XN_STATUS_OK)
    {
        ntk_throw_exception("Could not initialize the Session Manager :" + xnGetStatusString(rc));
    }
    m_session_manager->RegisterSession(this,
                                       &BodyEventDetectorSessionStart,
                                       &BodyEventDetectorSessionEnd);

    m_push_detector = new XnVPushDetector;
    m_push_detector->RegisterPush(this, &BodyEventDetectorPush_Pushed);
    m_session_manager->AddListener(m_push_detector);

    m_wave_detector = new XnVWaveDetector;
    m_wave_detector->SetFlipCount(5);
    m_wave_detector->RegisterWave(this, &BodyEventDetectorWave_Waved);
    m_session_manager->AddListener(m_wave_detector);

    // m_steady_detector = new XnVSteadyDetector;
    // m_steady_detector->RegisterSteady(this, &BodyEventDetectorSteady_Steady);
    // m_steady_detector->SetDetectionDuration(200);
    // m_steady_detector->SetMaximumVelocity(0.005);
    // m_session_manager->AddListener(m_steady_detector);

    m_circle_detector = new XnVCircleDetector;
    m_circle_detector->RegisterCircle(this, &BodyEventDetectorCircle_Circle);
    m_session_manager->AddListener(m_circle_detector);

    m_swipe_detector = new XnVSwipeDetector(true);
    m_swipe_detector->RegisterSwipeUp(this, &BodyEventDetectorSwipe_SwipeUp);
    m_swipe_detector->RegisterSwipeDown(this, &BodyEventDetectorSwipe_SwipeDown);
    m_swipe_detector->RegisterSwipeLeft(this, &BodyEventDetectorSwipe_SwipeLeft);
    m_swipe_detector->RegisterSwipeRight(this, &BodyEventDetectorSwipe_SwipeRight);
    // FIXME: disabled, using custom detector.
    // m_session_manager->AddListener(m_swipe_detector);

    if (m_pointer_smoothing_type != POINTER_SMOOTHING_DISABLED)
    {
        m_point_denoiser = new XnVPointDenoiser(12);
        m_session_manager->AddListener(m_point_denoiser);
        m_session_manager->AddListener(&m_raw_listener);
        m_point_denoiser->AddListener(this);
    }
    else
    {
        m_session_manager->AddListener(this);
    }

    initializeKalman();

    m_prev_pos_vector.resize(5);
}

void BodyEventDetector :: shutDown()
{
    m_context = 0;
    m_depth_generator = 0;

    delete_and_zero(m_session_manager);
    delete_and_zero(m_push_detector);
    delete_and_zero(m_swipe_detector);
    delete_and_zero(m_wave_detector);
    delete_and_zero(m_circle_detector);
    delete_and_zero(m_steady_detector);
    delete_and_zero(m_point_denoiser);
}

// Initialize Kalman filter from point noise filtering.
void BodyEventDetector :: initializeKalman()
{
    /*
     * 6 variables
     * x, Dx, y, Dy, z, Dz
     * with D the derivative.
     * This tracking assumes a constant speed model.
     */
    m_kalman = KalmanFilter(6, 6, 0);
    Mat1f state = Mat(6, 1, CV_32FC1);

    Mat1f process_noise (6, 1);
    Mat1f measurement (6, 1);
    Mat1f measurement_noise (6, 1);

    measurement = 0.f;

    /*
     * If there were only two variables, this generates a matrix
     * with the following form:
     * 1 1 0 0
     * 0 1 0 0
     * 0 0 1 1
     * 0 0 0 1
     */
    m_kalman.transitionMatrix = Mat1f(6, 6);
    setIdentity(m_kalman.transitionMatrix);
    for (int i = 0; i < 6; i += 2)
    {
        // Constant position model.
        // Put 1 here to assume a constant velocity model.
        m_kalman.transitionMatrix.at<float>(i,i+1) = 0;
    }
    // Derivates model is dx = dx' * 0.2
    // This means derivate is assumed to decrease with time.
    // This avoids a too strong inertia.
    m_kalman.transitionMatrix.at<float>(1,1) = 0.2f;
    m_kalman.transitionMatrix.at<float>(3,3) = 0.2f;
    m_kalman.transitionMatrix.at<float>(5,5) = 0.2f;

    setIdentity(m_kalman.measurementMatrix, 1);
    // Decrease or increase the process noise to change the
    // influence of new measures.
    setIdentity(m_kalman.processNoiseCov, 5e-2);
    setIdentity(m_kalman.measurementNoiseCov, 1);
    // Estimated empirically.
    m_kalman.measurementNoiseCov.at<float>(0,0) = 0.3918431f;
    m_kalman.measurementNoiseCov.at<float>(1,1) = 0.0236481f;
    m_kalman.measurementNoiseCov.at<float>(2,2) = 0.6588200f;
    m_kalman.measurementNoiseCov.at<float>(3,3) = 0.0848109f;
    m_kalman.measurementNoiseCov.at<float>(4,4) = 0.202737f;
    m_kalman.measurementNoiseCov.at<float>(5,5) = 0.0257873f;
    setIdentity(m_kalman.errorCovPost, 1e-1);
}

void BodyEventDetector :: update()
{
    m_session_manager->Update(m_context);
}

void BodyEventDetector :: sessionStartedCallback()
{
    ntk_dbg(1) << "Session started!";
    m_first_point_in_session = true;
}

void BodyEventDetector :: sessionFinishedCallback()
{
    ntk_dbg(1) << "Session finished :-(";
}

void BodyEventDetector :: sendEvent(const BodyEvent& event)
{
    foreach_idx(i, m_listeners)
    {
        m_listeners[i]->triggerEvent(event);
    }
}

void BodyEventDetector :: OnPrimaryPointCreate(const XnVHandPointContext* pContext, const XnPoint3D& ptSessionStarter)
{
    ntk_dbg(1) << "Primary point create";

    Point3f p3d = toPoint3f(pContext->ptPosition);
    m_prev_hand_points[0] = p3d;
    m_prev_hand_points[1] = p3d;
    m_prev_raw_pos[0] = p3d;
    m_prev_raw_pos[0] = p3d;
    uint64 timestamp = ntk::Time::getMillisecondCounter();
    m_prev_timestamps[0] = timestamp;
    m_prev_timestamps[1] = timestamp;
    m_tracked_user_id = pContext->nUserID;

    // derivatives are null initially.
    ((Mat1f)m_kalman.statePost)
            << m_prev_hand_points[0].x, 0,
            m_prev_hand_points[0].y, 0,
            m_prev_hand_points[0].z, 0;

    std::fill(stl_bounds(m_prev_pos_vector), p3d);
    m_bt = Point3f(0,0,0);
    m_st = p3d;
}

void BodyEventDetector :: OnPrimaryPointUpdate(const XnVHandPointContext* pContext)
{
}

static Point3f median(const std::vector<Point3f>& pos)
{
    std::vector<float> xvec(pos.size());
    std::vector<float> yvec(pos.size());
    std::vector<float> zvec(pos.size());
    foreach_idx(i, pos)
    {
        xvec[i] = pos[i].x;
        yvec[i] = pos[i].y;
        zvec[i] = pos[i].z;
    }
    std::sort(stl_bounds(xvec));
    std::sort(stl_bounds(yvec));
    std::sort(stl_bounds(zvec));
    Point3f p(xvec[xvec.size()/2], yvec[yvec.size()/2], zvec[zvec.size()/2]);
    return p;
}

void BodyEventDetector :: RawPointListener :: OnPointUpdate(const XnVHandPointContext* pContext)
{
}

void BodyEventDetector :: OnPointUpdate(const XnVHandPointContext* pContext)
{
    ntk_dbg(2) << "Point update";

    m_tracked_user_id = pContext->nUserID;

    uint64 timestamp = ntk::Time::getMillisecondCounter();
    // We want derivates in mm/ms
    float deltat = 1.0f / (timestamp - m_prev_timestamps[0]);
    if (ntk_isnan(deltat)) deltat = 1.0f/30.0f;
    if (deltat < 1) deltat = 1.0f/30.0f;

    float deltat_prev = 1.0f / (m_prev_timestamps[0] - m_prev_timestamps[1]);
    if (ntk_isnan(deltat_prev)) deltat_prev = 1.0f/30.0f;
    Point3f prev_derivate = (m_prev_hand_points[0]-m_prev_hand_points[1]) * (deltat_prev);

    Point3f raw_pos_3d = toPoint3f(pContext->ptPosition);
    Point3f raw_d3d = (raw_pos_3d-m_prev_raw_pos[0])*deltat;

    for(int i = m_prev_pos_vector.size()-1; i > 0; --i)
        m_prev_pos_vector[i] = m_prev_pos_vector[i-1];
    m_prev_pos_vector[0] = raw_pos_3d;
    // Point3f filtered_pos_3d = median(m_prev_pos_vector);
    Point3f filtered_pos_3d = raw_pos_3d;
    Point3f filtered_d3d = (filtered_pos_3d-m_prev_hand_points[0]) * deltat;

    // Default values are unfiltered.
    Point3f pos_3d = filtered_pos_3d;
    Point3f d3d = filtered_d3d;

    if (m_pointer_smoothing_type == POINTER_SMOOTHING_KALMAN)
    {
        m_kalman.predict();
        cv::Mat1f measurement(6,1);
        measurement << filtered_pos_3d.x, filtered_d3d.x,
                filtered_pos_3d.y, filtered_d3d.y,
                filtered_pos_3d.z, filtered_d3d.z;
        Mat1f new_state = m_kalman.correct(measurement);
        pos_3d = Point3f(new_state(0,0), new_state(2,0), new_state(4,0));
        d3d = Point3f(new_state(1,0), new_state(3,0), new_state(5,0));
    }
    else if (m_pointer_smoothing_type == POINTER_SMOOTHING_DOUBLE_EXPONENTIAL)
    {
        const float alpha = 0.23;
        const float beta = 0.18;
        m_bt = beta*(filtered_d3d) + (1.0-beta) * m_bt;
        m_st = alpha*filtered_pos_3d + (1.0f - alpha) * m_prev_hand_points[0];
        pos_3d = m_st + m_bt;
        d3d = (pos_3d - m_prev_hand_points[0]) * deltat;
    }

    XnPoint3D ni_2d;
    XnPoint3D xPos3d = toXnPoint3D(pos_3d);
    m_depth_generator->ConvertRealWorldToProjective(1, &xPos3d, &ni_2d);

    XnPoint3D ni_d2d;
    XnPoint3D xDev3d = toXnPoint3D(d3d);
    m_depth_generator->ConvertRealWorldToProjective(1, &xDev3d, &ni_d2d);

    HandPointUpdate event;
    event.timestamp = timestamp;
    event.pos_3d = pos_3d;
    event.pos_2d = toPoint3f(ni_2d);
    event.derivate_3d = d3d;
    event.derivate_2d = toPoint3f(ni_d2d);
    event.velocity = cv::norm(event.derivate_3d);
    event.acceleration = cv::norm(event.derivate_3d - prev_derivate) * deltat;
    m_prev_hand_points[1] = m_prev_hand_points[0];
    m_prev_hand_points[0] = pos_3d;
    m_prev_timestamps[1] = m_prev_timestamps[0];
    m_prev_timestamps[0] = timestamp;
    m_prev_raw_pos[1] = m_prev_raw_pos[0];
    m_prev_raw_pos[0] = raw_pos_3d;

    {
        QWriteLocker locker(&m_lock);
        m_last_hand_position = event.pos_3d;
        m_last_hand_position_2d = event.pos_2d;
    }

    foreach_idx(i, m_listeners)
    {
        m_listeners[i]->triggerHandPoint(event);
    }

    if (m_first_point_in_session)
    {
        // Send the first wave event, since it was the triggering event.
        BodyEvent event(BodyEvent::WaveEvent, 0, 0);
        sendEvent(event);
        m_first_point_in_session = false;
    }
}

}

