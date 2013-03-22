#ifndef   	NTK_STATS_HYPOTHESIS_TESTING_H_
# define   	NTK_STATS_HYPOTHESIS_TESTING_H_

# include <ntk/core.h>
# include <cmath>
# include <vector>
# include <map>

namespace ntk
{

  /**
   * Algorithms and examples come from "Non-parametric Statistics", Sigel,
   * McGraw-Hill Series in Psychology, 1956
   */

  std::vector< std::vector< std::vector<double> > > generate_mann_whitney_values();
  
  template <class T>
  double compute_mann_whitney_log_pfa(const std::vector<T>& d1, 
                                      const std::vector<T>& d2,
                                      double n1 = -1, double n2 = -1);
 
  /**
   * Probability that a samples of n element get a deviation dev by chance 
   * while underlying distribution moments are gdev and gmu4.
   */
  double deviation_log_pfa(double dev, double n, double gdev, double gmu4);

  template <class cdf_function>
  double cramer_von_mises_nw2(const std::map<double,double>& distrib, cdf_function f);

  template <class cdf_function>
  double kolmogorov_sqrt_n_Dn(const std::map<double,double>& distrib, cdf_function f);
  
  void estimate_abs_difference_distribution(std::vector<double>& output, 
                                            const std::vector<double>& d1, 
                                            const std::vector<double>& d2);
  
} // end of ntk

# include "hypothesis_testing.hxx"

#endif 	    /* !NTK_STATS_HYPOTHESIS_TESTING_H_ */
