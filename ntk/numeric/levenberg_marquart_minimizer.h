#ifndef NTK_NUMERIC_LEVENBERG_MARQUART_MINIMIZER_H
#define NTK_NUMERIC_LEVENBERG_MARQUART_MINIMIZER_H

#include <ntk/core.h>
#include <ntk/numeric/cost_function.h>

namespace ntk
{

class LevenbergMarquartMinimizer : public CostFunctionMinimizer
{
public:
  virtual void minimize(CostFunction& f, std::vector<double>& x);
  virtual void diagnoseOutcome(int debug_level = 2) const;

private:
  int m_info;
  int m_num_function_evaluations;
  int m_num_iterations;
  double m_start_error;
  double m_end_error;
};

} // ntk

#endif // NTK_NUMERIC_LEVENBERG_MARQUART_MINIMIZER_H
