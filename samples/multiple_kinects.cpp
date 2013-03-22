
#include <ntk/core.h>
#include <ntk/utils/debug.h>

#include <ntk/camera/openni_grabber.h>
#include <ntk/camera/rgbd_processor.h>
#include <ntk/utils/opencv_utils.h>

#include <QMessageBox>
#include <QApplication>

using namespace ntk;

int main(int argc, char** argv)
{    
    ntk::ntk_debug_level = 1;
	QApplication app(argc, argv);

    OpenniDriver ni_driver;

	if (ni_driver.numDevices() < 2)
	{
		QMessageBox::critical(0, "Error", "Less than two Kinect were detected.");
		exit(1);
	}

    OpenniGrabber grabber1(ni_driver, 0); // first id is 0
    OpenniGrabber grabber2(ni_driver, 1);

    grabber1.setTrackUsers(false);
    grabber2.setTrackUsers(false);

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
      post_processor.processImage(image1);

      grabber2.waitForNextFrame();
      grabber2.copyImageTo(image2);
      post_processor.processImage(image2);

      cv::Mat1b debug_depth_img1 = normalize_toMat1b(image1.depth());
      cv::Mat1b debug_depth_img2 = normalize_toMat1b(image2.depth());

      cv::Mat3b debug_color_img1 = image1.mappedRgb();
      cv::Mat3b debug_color_img2 = image2.mappedRgb();

      imshow("depth1", debug_depth_img1);
      imshow("color1", debug_color_img1);

      imshow("depth2", debug_depth_img2);
      imshow("color2", debug_color_img2);
      cv::waitKey(10);
    }
}
