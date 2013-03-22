
#include <ntk/utils/debug.h>
#include <ntk/numeric/utils.h>
#include <ntk/stats/distributions.h>
#include <ntk/stats/hypothesis_testing.h>
#include <ntk/utils/xml_serializable.h>
// #include <gsl/gsl_cdf.h>

using namespace ntk;

#if 0
void test_distribution()
{
  EmpiricalDistribution d;
  for (int i = 0; i < 1; ++i) d.addValue(10);
  for (int i = 0; i < 5; ++i) d.addValue(20);
  for (int i = 0; i < 5; ++i) d.addValue(50);
  for (int i = 0; i < 3; ++i) d.addValue(70);
  for (int i = 0; i < 1; ++i) d.addValue(80);

  XMLNode e;
  d.fillXmlElement(e);

  EmpiricalDistribution d2;
  d2.loadFromXmlElement(e);

  ntk_dbg_print(d.cdf(-10), 0);
  ntk_dbg_print(d.cdf(0), 0); 
  ntk_dbg_print(d.cdf(10), 0);
  ntk_dbg_print(d.cdf(15), 0);
  ntk_dbg_print(d.cdf(35), 0);
  ntk_dbg_print(d.cdf(90), 0);
  ntk_dbg_print(d.cdf(100), 0);
  ntk_dbg_print(d.cdf(200), 0);

  ntk_dbg_print(d2.cdf(-10), 0);
  ntk_dbg_print(d2.cdf(0), 0); 
  ntk_dbg_print(d2.cdf(10), 0);
  ntk_dbg_print(d2.cdf(15), 0);
  ntk_dbg_print(d2.cdf(35), 0);
  ntk_dbg_print(d2.cdf(90), 0);
  ntk_dbg_print(d2.cdf(100), 0);
  ntk_dbg_print(d2.cdf(200), 0);  
}
#endif

void test_abs_difference_distribution()
{
  std::vector<double> d1 (4);
  std::vector<double> d2 (4);
  std::vector<double> output;
  
  d1[0] = 0.2; d1[1] = 0.1; d1[2] = 0.3; d1[3] = 0.4;
  d2[0] = 0.4; d2[1] = 0.1; d2[2] = 0.2; d2[3] = 0.3;
  estimate_abs_difference_distribution(output, d1, d2);
  for (int i = 0; i < (int)output.size(); ++i)
  {
    ntk_dbg(1) << i << ": " << output[i];
  }
}

int main()
{
  ntk::ntk_debug_level = 1;
  test_abs_difference_distribution();
  return 0;
}
