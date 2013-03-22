#ifndef NTK_NUMERIC_DIFFERENTIAL_EVOLUTION_MINIMIZER_H
#define NTK_NUMERIC_DIFFERENTIAL_EVOLUTION_MINIMIZER_H

#include <ntk/core.h>
#include <ntk/numeric/cost_function.h>

namespace ntk
{

class DifferentialEvolutionMinimizer : public CostFunctionMinimizer
{
public:
  DifferentialEvolutionMinimizer(int population_size,
                                 int max_generations,
                                 const std::vector<double>& min_values,
                                 const std::vector<double>& max_values)
    : m_population_size(population_size),
      m_max_generations(max_generations),
      m_min_values(min_values),
      m_max_values(max_values)
  {
  }

public:
  virtual void minimize(CostFunction& f, std::vector<double>& x);

private:
  int m_population_size;
  int m_max_generations;
  std::vector<double> m_min_values;
  std::vector<double> m_max_values;
};

} // ntk

#endif // NTK_NUMERIC_DIFFERENTIAL_EVOLUTION_MINIMIZER_H
