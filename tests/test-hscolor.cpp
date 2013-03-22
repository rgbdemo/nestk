
#include <ntk/image/color_model.h>
#include <opencv2/highgui/highgui.hpp>
#include <ntk/utils/opencv_utils.h>
#include <iostream>

using namespace ntk;
using namespace cv;

int main(int argc, char** argv)
{
  if (argc != 3)
  {
    std::cerr << "Usage: test-hscolor model_image.png new_image.png" << std::endl;
    exit(1);
  }

  const char* ref_image_name = argv[1];
  const char* new_image_name = argv[2];

  cv::Mat3b ref_image = imread(ref_image_name);
  cv::Mat3b new_image = imread(new_image_name);

  HSColorModel model;
  model.build(ref_image, Mat1b() /* empty mask, use all image */);
  cv::Mat3b display_img;
  model.show(display_img);
  imshow("Histogram", display_img);
  cv::waitKey(0);

  cv::Mat1f likelihood_image;
  model.backProject(new_image, likelihood_image);
  cv::Mat1b output;
  imwrite_normalized("likehood.png", likelihood_image);
  threshold(likelihood_image, output, 0.2, 255, cv::THRESH_BINARY);
  imwrite("output.png", output);
}
