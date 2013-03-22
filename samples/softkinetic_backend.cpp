
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
