#ifndef NTK_COLORMODEL_H_
# define NTK_COLORMODEL_H_

# include <ntk/core.h>

// TODO: use mixture of gaussians?
class HSColorModel
{
public:
  HSColorModel() {}
  ~HSColorModel() { }

public:
  void show(cv::Mat3b& display) const;
  void build(const cv::Mat3b& model_image, const cv::Mat1b& mask);
  double likelihood(int h_value, int s_value) const;
  void backProject(const cv::Mat3b& bgr_image, cv::Mat1f& likelihood_image) const;

private:
  cv::Mat_<float> m_histogram;
};

#endif // ndef NTK_COLORMODEL_H_
