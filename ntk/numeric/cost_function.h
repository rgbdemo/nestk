#ifndef NTK_NUMERIC_COST_FUNCTION_H
#define NTK_NUMERIC_COST_FUNCTION_H

#include <ntk/core.h>
#include <vector>

namespace ntk
{

class CostFunction
{
public:
  // Initialize a cost function providing the number of variables
  // of the function to optimize and the number of errors it provides.
  CostFunction(int func_input_dimension, int func_output_dimension)
    : m_input_dim(func_input_dimension),
      m_output_dim(func_output_dimension)
  {}

  virtual void evaluate(const std::vector<double>& input, std::vector<double>& output) const = 0;
  double outputNorm(const std::vector<double>& input) const;
  double normalizedOutputNorm(const std::vector<double>& input) const
  { return outputNorm(input) / outputDimension(); }

  int inputDimension() const { return m_input_dim; }
  int outputDimension() const { return m_output_dim; }

private:
  int m_input_dim;
  int m_output_dim;
};

class CostFunctionMinimizer
{
public:
  // Minimize the given function taking x as initial value.
  // When returning, x will contain the found minima.
  virtual void minimize(CostFunction& f, std::vector<double>& x) = 0;
  virtual void diagnoseOutcome(int debug_level) const {};
};

}

#endif // NTK_NUMERIC_COST_FUNCTION_H
