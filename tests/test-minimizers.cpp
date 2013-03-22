
#include <ntk/ntk.h>
#include <ntk/numeric/levenberg_marquart_minimizer.h>
#include <ntk/numeric/differential_evolution_minimizer.h>
#include <ntk/utils/time.h>

using namespace ntk;
using namespace cv;

// Simple function from R2 => R2
// f(x,y) = [ x*(y+10) ; 1 / (x*y+1) + y ]
// The minimum is 0 at x=0 y=-1
struct CostFunction1 : public CostFunction
{
  // two input, two outputs
  CostFunction1() : CostFunction(2, 2)
  {}

  virtual void evaluate(const std::vector<double>& input, std::vector<double>& output) const
  {
    output[0] = input[0]*(input[1]+10);
    output[1] = 1.0 / (input[0]*input[1]+1) + input[1];
  }
};

int main()
{
  ntk::ntk_debug_level = 1;
  CostFunction1 f;

  LevenbergMarquartMinimizer lvq_minimizer;
  std::vector<double> x(2);
  x[0] = 1;
  x[1] = 1;
  ntk_dbg_print(f.outputNorm(x), 1);
  TimeCount tlvq("lvq");
  lvq_minimizer.minimize(f, x);
  tlvq.stop();
  ntk_dbg_print(f.outputNorm(x), 1);
  ntk_dbg_print(x[0], 0);
  ntk_dbg_print(x[1], 0);
  ntk_ensure(std::abs(x[0]) < 1e-2, "x[0] should be 0");
  ntk_ensure(std::abs(x[1]+1) < 1e-2, "x[1] should be -1");

  x[0] = 1;
  x[1] = 1;
  std::vector<double> min_values(2, -100), max_values(2, 100);
  DifferentialEvolutionMinimizer de_minimizer(1000, 50, min_values, max_values);
  ntk_dbg_print(f.outputNorm(x), 1);
  TimeCount tde("tde");
  de_minimizer.minimize(f, x);
  tde.stop();
  ntk_dbg_print(f.outputNorm(x), 1);
  ntk_dbg_print(x[0], 0);
  ntk_dbg_print(x[1], 0);
  ntk_ensure(std::abs(x[0]) < 1e-2, "x[0] should be 0");
  ntk_ensure(std::abs(x[1]+1) < 1e-2, "x[1] should be -1");

  return 0;
}
