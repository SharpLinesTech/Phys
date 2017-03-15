#ifndef PHYS_SEQUENTIAL_INPUT_SOLVER_CONFIG_H
#define PHYS_SEQUENTIAL_INPUT_SOLVER_CONFIG_H

namespace phys {
namespace seqi_solver {
template <typename CFG>
struct Config {
  using real_t = typename CFG::real_t;

  // Number of iterations to perform.
  int iterations = 10;

  // Number of iterations to perform for penetration resolving
  int penetration_iterations = 10;

  real_t residual_threshold = 0.0f;

  real_t erp = real_t(0.2);
  real_t cfm = real_t(0.0);

  real_t split_impulse_penetration_threshold = real_t(-0.04);
  real_t split_impulse_turn_erp = real_t(0.1);
};
}
}

#endif