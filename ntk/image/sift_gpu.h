#ifndef NTK_IMAGE_SIFT_GPU_H
# define NTK_IMAGE_SIFT_GPU_H

#include <ntk/core.h>
#include "siftgpu/SiftGPU.h"
#include <opencv2/features2d/features2d.hpp>
#include <QLocalServer>
#include <QLocalSocket>

namespace ntk
{

  SiftGPU* getSiftGPUInstance();

  class GPUSiftDetector
  {
  public:
    int descriptorSize() const { return 128; }

    void operator()(const cv::Mat1b& img, const cv::Mat& mask,
                    std::vector<cv::KeyPoint>& keypoints,
                    std::vector<float>& descriptors) const;
  };

  class GPUSiftServer
  {
  public:
    void run();
    void stop();
    static bool isSupported();
    int descriptorSize() const { return 128; }

  private:
    void receiveRequest();
    void detectKeypoints();
    void sendPoints();
    void quit();

  private:
    QLocalServer m_server;
    QLocalSocket* m_current_socket;
    cv::Mat1b m_current_image;
    std::vector<cv::KeyPoint> m_current_keypoints;
    std::vector<float> m_current_descriptors;
    GPUSiftDetector m_detector;
    int m_pid;
  };

  class GPUSiftClient
  {
  public:
    int descriptorSize() const { return 128; }

    void operator()(const cv::Mat1b& img, const cv::Mat& mask,
                    std::vector<cv::KeyPoint>& keypoints,
                    std::vector<float>& descriptors) const;
  };

} // ntk

#endif // !NTK_IMAGE_SIFT_GPU_H
