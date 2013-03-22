
#include <ntk/image/sift_gpu.h>
#include <ntk/ntk.h>

int main(int argc, char** argv)
{
  ntk_ensure(argc == 2, "Usage: test-sift-gpu image");

  SiftGPU* sift = ntk::getSiftGPUInstance();
  ntk_assert(sift, "Could not get sift gpu");

  std::vector<float > descriptors;
  std::vector<SiftGPU::SiftKeypoint> keypoints;

  cv::Mat1b image = cv::imread(argv[1], 0);
  ntk_ensure(image.data, "Could not load image");

  for (int i = 0; i < 100; ++i)
  {
    int t1 = cv::getTickCount();

    if(!sift->RunSIFT(image))
      ntk_throw_exception("Could not run sift on test.jpg");

    int n_features = sift->GetFeatureNum();
    std::cout << "Nb points: " << n_features << std::endl;

    keypoints.resize(n_features);
    descriptors.resize(128*n_features);

    sift->GetFeatureVector(&keypoints[0], &descriptors[0]);

    int t2 = cv::getTickCount();
    std::cout << 1000.0*(t2-t1)/cv::getTickFrequency() << " ms" << std::endl;
  }

  return 0;
}
