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

using namespace cv;
using namespace ntk;

int main(int argc, char **argv)
{
  ntk::ntk_debug_level = 1;
  QApplication app (argc, argv);
  QDir::setCurrent(QApplication::applicationDirPath());

  NiteRGBDGrabber grabber;
  grabber.initialize();
  grabber.start();

  RGBDImage image;
  Skeleton skeleton;

  namedWindow("depth");
  namedWindow("color");

  while (true)
  {
    grabber.waitForNextFrame();

    // Lock the grabber to ensure data do not change in grab thread.
    grabber.acquireReadLock();
    grabber.currentImage().copyTo(image);
    grabber.skeleton().copyTo(skeleton);
    grabber.releaseReadLock();

    cv::Mat1b debug_depth_img = normalize_toMat1b(image.depth());
    skeleton.drawOnImage(debug_depth_img);
    cv::Mat3b debug_color_img;
    image.mappedRgb().copyTo(debug_color_img);
    skeleton.drawOnImage(debug_color_img);

    imshow("depth", debug_depth_img);
    imshow("color", debug_color_img);
    cv::waitKey(10);
  }

  return app.exec();
}
