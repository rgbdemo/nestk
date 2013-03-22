
#include <ntk/image/sift_gpu.h>
#include <ntk/utils/debug.h>

using namespace cv;
using namespace ntk;

int main(int argc, char** argv)
{
  ntk::ntk_debug_level = 1;

  GPUSiftServer server;
  server.run();

  GPUSiftClient client;

  for (int i = 1; i < argc; ++i)
  {
    cv::Mat1b img = imread(argv[i], 0);
    ntk_ensure(img.data, "Could not read image");
    std::vector<KeyPoint> keypoints;
    std::vector<float> descriptors;
    client(img, Mat(), keypoints, descriptors);
    cv::Mat3b debug_img;
    drawKeypoints(img, keypoints, debug_img);
    imshow("sift", debug_img);
    waitKey(0);
  }

  server.stop();
}
