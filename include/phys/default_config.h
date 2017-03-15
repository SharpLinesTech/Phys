#ifndef PHYS_DEFAULT_CONFIG_H
#define PHYS_DEFAULT_CONFIG_H

#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"

namespace phys {

struct DefaultConfig {
  enum { max_contact_points_per_collision = 4 };

  using real_t = float;
  using vec3_t = glm::vec3;
  using mat3x3_t = glm::mat3;
  using mat4x4_t = glm::mat4;

  // N.B. GLM quats are oredered as wxyz, so any replacement type must follow
  // this.
  using quat_t = glm::quat;

  using collision_mask_t = uint8_t;  // switch to uint16_t, uint32_t, or
                                     // uint64_t if you need more than 8 masks.
};
}

#include "phys/collision/broadphase/axis_sweep.h"
#include "phys/dynamics/solver/sequential_impulse/solver.h"

namespace phys {
template <typename CFG>
struct DefaultAlgos {
  using Solver = phys::seqi_solver::Solver<CFG>;
  using Broadphase = phys::col::AxisSweepBroadphase<CFG>;
};
}

#endif