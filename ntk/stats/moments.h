#ifndef   	NTK_STATS_MOMENTS_H_
# define   	NTK_STATS_MOMENTS_H_

# include <ntk/core.h>

# include <memory>
# include <string>
# include <sstream>
# include <fstream>
# include <vector>

namespace ntk
{

  double median(std::vector<double>& values);

  template <class T>
  void values_moments(const T& begin, const T& end, unsigned np, double* mean, double* dev);

  template <class T>
  void distrib_moments(const T& distrib, double& mean, double& dev);

  template <class T>
  void distrib_moments(const T& distrib, double& mean, double& dev, double& mu4);

  std::vector<double> distrib_to_bimodal_linear(const std::vector<double>& histogram);

  std::vector<double>
      fill_linear_values_from_control_points(const std::vector<cv::Point2f>& points);

  void keep_local_maxima(std::vector<cv::Point2f>* maxima, const std::vector<double>& histogram);
  std::vector<cv::Point2f> make_unimodal_distribution(const std::vector<cv::Point2f>& points);

  double recursive_mean(double previous_mean, double previous_size, double new_value);
  double recursive_variance(double previous_variance, double new_mean, 
			    double previous_size, double new_value);

} // end of ntk

#include "moments.hxx"

#endif	    /* !NTK_STATS_MOMENTS_H_ */
