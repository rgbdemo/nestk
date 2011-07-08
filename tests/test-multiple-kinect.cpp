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

#include <ntk/core.h>
#include <ntk/utils/debug.h>

#include <ntk/camera/openni_grabber.h>
#include <ntk/camera/rgbd_processor.h>
#include <ntk/utils/opencv_utils.h>

using namespace ntk;

int main()
{    
    OpenniGrabber grabber1(0);
    OpenniGrabber grabber2(1);

    grabber1.connectToDevice();
    grabber2.connectToDevice();

    grabber1.start();
    grabber2.start();

    RGBDImage image1, image2;
    OpenniRGBDProcessor post_processor;

    while (true)
    {
      // Wait for a new frame, get a local copy and postprocess it.
      grabber1.waitForNextFrame();
      grabber1.copyImageTo(image1);

      grabber2.waitForNextFrame();
      grabber2.copyImageTo(image2);
      post_processor.processImage(image2);

      cv::Mat1b debug_depth_img1 = normalize_toMat1b(image1.depth());
      cv::Mat1b debug_depth_img2 = normalize_toMat1b(image2.depth());

      cv::Mat3b debug_color_img1 = image1.mappedRgb();
      cv::Mat3b debug_color_img2 = image2.mappedRgb();

      cv::Mat3b debug_users1;
      cv::Mat3b debug_users2;
      image1.fillRgbFromUserLabels(debug_users1);
      image2.fillRgbFromUserLabels(debug_users2);

      imshow("depth1", debug_depth_img1);
      imshow("color1", debug_color_img1);
      imshow("users1", debug_users1);

      imshow("depth2", debug_depth_img2);
      imshow("color2", debug_color_img2);
      imshow("users2", debug_users2);
      cv::waitKey(10);
    }
}
