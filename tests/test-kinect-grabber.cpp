
#include <ntk/camera/freenect_grabber.h>
#include <ntk/camera/rgbd_processor.h>
#include <ntk/utils/opencv_utils.h>

#include <QApplication>

using namespace ntk;
using namespace cv;

int main(int argc, char** argv)
{
  // QApplication app(argc, argv);

  FreenectGrabber grabber;
  grabber.connectToDevice();
  grabber.start();

  // Set camera tilt.
  grabber.setTiltAngle(15);

  // New opencv window
  namedWindow("color");
  namedWindow("ir");
  namedWindow("depth");
  namedWindow("depth_as_color");

  // Tell the processor to transform raw depth into meters using linear coefficients.
  FreenectRGBDProcessor processor;

  RGBDImage current_frame;
  cv::Mat3b depth_as_color;
  while (true)
  {
    grabber.waitForNextFrame();
    grabber.copyImageTo(current_frame);
    processor.processImage(current_frame);

    int fps = grabber.frameRate();
    cv::putText(current_frame.rgbRef(),
                cv::format("%d fps", fps),
                Point(10,20), 0, 0.5, Scalar(255,0,0,255));

    // Display the image
    imshow("color", current_frame.rgb());
    imshow("ir", current_frame.intensity());

    // Show depth as normalized gray scale
    imshow_normalized("depth", current_frame.depth());

    // Compute color encoded depth.
    compute_color_encoded_depth(current_frame.depth(), depth_as_color);
    imshow("depth_as_color", depth_as_color);

    unsigned char c = cv::waitKey(10);
    printf("c:%d\n", c);
    if (c == 'f')
      grabber.setIRMode(!grabber.irModeEnabled());
  }
}
