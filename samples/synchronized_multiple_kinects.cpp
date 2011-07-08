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
#include <ntk/camera/multiple_grabber.h>
#include <ntk/camera/rgbd_processor.h>
#include <ntk/utils/opencv_utils.h>

using namespace ntk;

int main()
{    
    OpenniGrabber grabber1(0); // first id is 0
    OpenniGrabber grabber2(1);

    // Will wait for all grabbers to have provided their image and
    // deliver a vector of images.
    MultipleGrabber grabber;
    grabber.addGrabber(&grabber1);
    grabber.addGrabber(&grabber2);

    grabber.connectToDevice();
    grabber.start();

    std::vector<RGBDImage> images;
    OpenniRGBDProcessor post_processor;

    while (true)
    {
        // Wait for a new frame, get a local copy and postprocess it.
        grabber.waitForNextFrame();
        grabber.copyImagesTo(images);

        ntk_ensure(images.size() == 2, "This program needs two kinects.");

        post_processor.processImage(images[0]);
        post_processor.processImage(images[1]);

        cv::Mat1b debug_depth_img1 = normalize_toMat1b(images[0].depth());
        cv::Mat1b debug_depth_img2 = normalize_toMat1b(images[1].depth());

        cv::Mat3b debug_color_img1 = images[0].mappedRgb();
        cv::Mat3b debug_color_img2 = images[1].mappedRgb();

        cv::Mat3b debug_users1;
        cv::Mat3b debug_users2;
        images[0].fillRgbFromUserLabels(debug_users1);
        images[1].fillRgbFromUserLabels(debug_users2);

        imshow("depth1", debug_depth_img1);
        imshow("color1", debug_color_img1);
        imshow("users1", debug_users1);

        imshow("depth2", debug_depth_img2);
        imshow("color2", debug_color_img2);
        imshow("users2", debug_users2);
        cv::waitKey(10);
    }
}
