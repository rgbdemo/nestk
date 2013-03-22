
#include "differential_evolution_minimizer.h"
#include "differential_evolution_solver.h"

#include <ntk/utils/debug.h>
#include <ntk/numeric/utils.h>

#include <unsupported/Eigen/NonLinearOptimization>

using namespace Eigen;
using namespace ntk;

namespace
{

class CostFunctionSolver : public DifferentialEvolutionSolver
{
public:
  CostFunctionSolver(ntk::CostFunction& cost_function, int population_size)
    : DifferentialEvolutionSolver(cost_function.inputDimension(), population_size),
      m_cost_function(cost_function),
      m_current_input(cost_function.inputDimension()),
      m_current_output(cost_function.outputDimension()),
      m_input_dim(cost_function.inputDimension()),
      m_output_dim(cost_function.outputDimension())
  {}

  virtual double EnergyFunction(double trial[],bool &bAtSolution)
  {
    std::copy(trial, trial + m_input_dim, m_current_input.begin());
    m_cost_function.evaluate(m_current_input, m_current_output);
    double error = 0;
    foreach_idx(i, m_current_output)
    {
      error += ntk::math::sqr(m_current_output[i]);
    }
    return error;
  }

private:
  ntk::CostFunction& m_cost_function;
  std::vector<double> m_current_input;
  std::vector<double> m_current_output;
  int m_input_dim;
  int m_output_dim;
};

}

namespace ntk
{

void DifferentialEvolutionMinimizer :: minimize(ntk::CostFunction &f,
                                                std::vector<double> &x)
{
  ntk_assert(m_min_values.size() == f.inputDimension()
             && m_max_values.size() == f.inputDimension(),
             "min/max values must have input dimension size.");
  CostFunctionSolver solver (f, m_population_size);
  solver.Setup(&m_min_values[0], &m_max_values[0],
               DifferentialEvolutionSolver::stBest1Exp,0.8,0.75);
  solver.Solve(m_max_generations);
  double *solution = solver.Solution();
  std::copy(solution, solution + f.inputDimension(), x.begin());
  ntk_dbg_print(solver.Energy(), 1);
}

} // ntk
