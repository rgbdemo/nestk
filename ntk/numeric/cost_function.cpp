
#include "cost_function.h"

namespace ntk
{

double CostFunction :: outputNorm(const std::vector<double>& input) const
{
  std::vector<double> output(m_output_dim);
  evaluate(input, output);
  double err = 0;
  foreach_idx(i, output)
  {
    err += output[i]*output[i];
  }
  err = sqrt(err);
  return err;
}

}
