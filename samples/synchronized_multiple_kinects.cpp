
#include <ntk/core.h>
#include <ntk/utils/debug.h>

#include <ntk/camera/openni_grabber.h>
#include <ntk/camera/multiple_grabber.h>
#include <ntk/camera/rgbd_processor.h>
#include <ntk/utils/opencv_utils.h>

#include <QMessageBox>
#include <QApplication>

using namespace ntk;

int main(int argc, char** argv)
{    
	QApplication app(argc, argv);

    OpenniDriver ni_driver;

	if (ni_driver.numDevices() < 2)
	{
		QMessageBox::critical(0, "Error", "Less than two Kinect were detected.");
		exit(1);
	}

    OpenniGrabber grabber1(ni_driver, 0); // first id is 0
    OpenniGrabber grabber2(ni_driver, 1);

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
