
#include <ntk/camera/opencv_grabber.h>
#include <ntk/camera/rgbd_processor.h>

using namespace ntk;
using namespace cv;

int main()
{
  // Create a grabber with the specified image size.
  OpencvGrabber grabber(cv::Size(640, 480));

  // Connect with the camera, opencv id is 0
  grabber.connectToDevice(0 /* camera id */);

  // Start the grabbing thread
  grabber.start();

  // New opencv window
  namedWindow("color");
  RGBDProcessor processor;
  RGBDImage current_frame;

  while (true)
  {
    // Wait for the grabber to have a new image
    grabber.waitForNextFrame();

    // Get the new image
    grabber.copyImageTo(current_frame);

    // Process the raw data
    processor.processImage(current_frame);

    // Show the framerate
    int fps = grabber.frameRate();
    cv::putText(current_frame.rgbRef(),
                cv::format("%d fps", fps),
                Point(10,20), 0, 0.5, Scalar(255,0,0,255));

    // Display the image
    imshow("color", current_frame.rgb());
    cv::waitKey(10);
  }
}
