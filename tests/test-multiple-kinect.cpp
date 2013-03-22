
#include <ntk/core.h>
#include <ntk/utils/debug.h>

#include <ntk/camera/openni_grabber.h>
#include <ntk/camera/rgbd_processor.h>
#include <ntk/utils/opencv_utils.h>

using namespace ntk;

int main()
{
    ntk::ntk_debug_level = 1;
    OpenniDriver driver;
    ntk_dbg_print(driver.numDevices(), 1);

    OpenniGrabber grabber1(driver, 0);
    OpenniGrabber grabber2(driver, 1);

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
