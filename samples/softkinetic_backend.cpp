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
#include <ntk/camera/softkinetic_grabber.h>

#include <QApplication>
#include <QDir>
#include <QMutex>

using namespace cv;
using namespace ntk;

namespace opt
{
}

int main(int argc, char **argv)
{
    // Parse command line options.
    arg_base::set_help_option("-h");
    arg_parse(argc, argv);

    // Set debug level to 1.
    ntk::ntk_debug_level = 1;

    // Set current directory to application directory.
    // This is to find Nite config in config/ directory.
    QApplication app (argc, argv);
    QDir::setCurrent(QApplication::applicationDirPath());

    // Declare the grabber.
    SoftKineticGrabber grabber;

    // Start the grabber.
    grabber.connectToDevice();
    grabber.start();

    // Holder for the current image.
    RGBDImage image;

    namedWindow("depth");
    namedWindow("color");

    char last_c = 0;
    while (true && (last_c != 27))
    {
        // Wait for a new frame, get a local copy and postprocess it.
        grabber.waitForNextFrame();
        grabber.copyImageTo(image);

        // Prepare the depth view.
        cv::Mat1b debug_depth_img = normalize_toMat1b(image.rawDepth());

        // Prepare the color view with skeleton and handpoint.
        cv::Mat3b debug_color_img;
        image.rawRgb().copyTo(debug_color_img);

        imshow("depth", debug_depth_img);
        imshow("color", debug_color_img);
        last_c = (cv::waitKey(10) & 0xff);
    }
    grabber.stop();
}
