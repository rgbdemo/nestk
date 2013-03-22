
#include <ntk/ntk.h>
#include <ntk/stats/estimation.h>
#include <ntk/stats/distributions.h>
#include <ntk/stats/histogram.h>

#include "test_common.h"

using namespace ntk;

bool test_tail_extrapolator()
{
  bool ok = true;

  EmpiricalDistribution distrib;
  std::vector<double> values (21);
  for (int i = 0; i < 21; ++i)
  {
    distrib.addValue(i);
    values[i] = 1;
  }

  NTK_TEST_FLOAT_EQ(distrib.logCdf(-10), -102.545f);
  NTK_TEST_FLOAT_EQ(distrib.logCdf(-20), -202.545f);
  NTK_TEST_FLOAT_EQ(distrib.logCdf(-50), -502.545f);

  distrib.saveLogCdfAsPlot("/tmp/distrib.plot");

  ParetoTailExtrapolator extrapolator (distrib);
  NTK_TEST_FLOAT_EQ(extrapolator.extrapolateLogCdf(-100), -103.045f);
  return true;
}

bool test_mean()
{
  EmpiricalDistribution distrib;
  std::vector<double> values (21);
  for (int i = 0; i < 21; ++i)
  {
    distrib.addValue(i);
    values[i] = 1;
  }

  {
    EmpiricalDistribution mean_distrib_for_5;
    mean_empirical_distribution(mean_distrib_for_5, distrib, 5);
    NTK_TEST_FLOAT_EQ(mean_distrib_for_5.mean(), 9.80706f);
    NTK_TEST_FLOAT_EQ(mean_distrib_for_5.logCdf(3), -5.62524f);
  }

  {
    EmpiricalDistribution mean_distrib_for_5;
    mean_empirical_distribution(mean_distrib_for_5, values, 5);
    NTK_TEST_FLOAT_EQ(mean_distrib_for_5.logCdf(3), -5.51626f);
  }

  EmpiricalMeanDistribution mean_distrib (9);
  mean_distrib.reset(distrib);
  ntk_dbg_print(mean_distrib.logCdf(1, 3), 0);
  return true;
}

bool test_tail_estimation(const char* filename)
{
  EmpiricalDistribution distrib(0.01f);
  QFile xml_file(filename);
  distrib.loadFromXml(xml_file);
  distrib.saveLogCdfAsPlot("distrib.plot");
  return true;
}

int main(int argc, char** argv)
{
  ntk::ntk_debug_level = 1;

  bool ok = true;

  ok &= test_tail_extrapolator();
  ok &= test_mean();
  //if (argc > 1) ok &= test_tail_estimation(argv[1]);
  return ok != true;
}
