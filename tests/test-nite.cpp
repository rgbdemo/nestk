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

#include <ntk/ntk.h>
#include <ntk/utils/debug.h>
#include <ntk/camera/nite_rgbd_grabber.h>
#include <ntk/gesture/body_event.h>

#include <QApplication>
#include <QDir>
#include <QMutex>

using namespace cv;
using namespace ntk;

class MyBodyListener : public BodyEventListener
{
public:
  MyBodyListener() : m_last_hand_pos_2d(0,0,0)
  {

  }

  virtual void triggerEvent(const BodyEvent &event)
  {
    switch (event.kind)
    {
    case BodyEvent::PushEvent:
      ntk_dbg(1) << "PushEvent detected!";
      break;
    case BodyEvent::WaveEvent:
      ntk_dbg(1) << "WaveEvent detected!";
      break;
    case BodyEvent::SteadyEvent:
      ntk_dbg(1) << "SteadyEvent detected!";
      break;
    };
  }

  virtual void triggerHandPoint(const HandPointUpdate &hand_point)
  {
    QMutexLocker locker(&m_lock);
    m_last_hand_pos_2d = hand_point.pos_2d;
  }

  cv::Point3f getLastHandPosInImage() const
  {
    QMutexLocker locker(&m_lock);
    return m_last_hand_pos_2d;
  }

private:
  mutable QMutex m_lock;
  Point3f m_last_hand_pos_2d;
};

int main(int argc, char **argv)
{
  ntk::ntk_debug_level = 1;
  QApplication app (argc, argv);
  QDir::setCurrent(QApplication::applicationDirPath());

  MyBodyListener body_event_listener;
  BodyEventDetector detector;
  detector.addListener(&body_event_listener);

  NiteRGBDGrabber grabber;
  grabber.setBodyEventDetector(&detector);

  grabber.initialize();
  grabber.start();

  RGBDImage image;

  namedWindow("depth");
  namedWindow("color");

  while (true)
  {
    grabber.waitForNextFrame();

    // Lock the grabber to ensure data do not change in grab thread.
    grabber.acquireReadLock();
    grabber.currentImage().copyTo(image);
    grabber.releaseReadLock();

    cv::Point3f handpoint = body_event_listener.getLastHandPosInImage();

    cv::Mat1b debug_depth_img = normalize_toMat1b(image.depth());
    if (image.skeleton())
      image.skeleton()->drawOnImage(debug_depth_img);
    circle(debug_depth_img, Point(handpoint.x, handpoint.y), 5, Scalar(0, 255, 255));

    cv::Mat3b debug_color_img;
    image.mappedRgb().copyTo(debug_color_img);
    if (image.skeleton())
      image.skeleton()->drawOnImage(debug_color_img);
    circle(debug_color_img, Point(handpoint.x, handpoint.y), 5, Scalar(0, 255, 255));

    imshow("depth", debug_depth_img);
    imshow("color", debug_color_img);
    cv::waitKey(10);
  }

  return app.exec();
}
