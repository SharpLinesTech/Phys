#ifndef PHYS_SEQUENTIAL_INPUT_SOLVER_BODY_H
#define PHYS_SEQUENTIAL_INPUT_SOLVER_BODY_H

#include "phys/dynamics/dynamic_body.h"
#include "phys/math_types/transform.h"

namespace phys {
namespace seqi_solver {

template <typename CFG>
struct Body {
  using vec3_t = typename CFG::vec3_t;
  using real_t = typename CFG::real_t;

  Body() {}
  Body(DynamicBodyData<CFG>* tgt, real_t dt) {
    target = tgt;
    world_transform = *target->transform;

    inv_mass = real_t(1) / target->mass_;
    linear_velocity = tgt->linear_velocity_;
    angular_velocity = tgt->angular_velocity_;

    applied_force_impulse = target->net_force_ * inv_mass * dt;
    applied_torque_impulse =
        target->net_torque_ * target->inv_inertia_tensor_world_ * inv_mass * dt;
  }

  DynamicBodyData<CFG>* target = nullptr;

  Transform<CFG> world_transform;

  real_t inv_mass = real_t(0);
  vec3_t linear_velocity = {0, 0, 0};

  vec3_t angular_velocity = {0, 0, 0};

  vec3_t applied_force_impulse = {0, 0, 0};
  vec3_t applied_torque_impulse = {0, 0, 0};

  vec3_t delta_v = {0, 0, 0};
  vec3_t delta_w = {0, 0, 0};

  // Contact penetration related.
  vec3_t push_vel = {0, 0, 0};
  vec3_t turn_vel = {0, 0, 0};
  bool push_applied = false;

  void applyPushImpulse(vec3_t const& lin, vec3_t const& ang, real_t mag) {
    push_vel += lin * mag;
    turn_vel += ang * mag;
    push_applied = true;
  }

  void applyImpulse(vec3_t const& lin, vec3_t const& ang, real_t mag) {
    delta_v += lin * mag;
    delta_w += ang * mag;
  }

  vec3_t getRelativeVelocity(vec3_t const& p) const {
    return linear_velocity + applied_force_impulse +
           cross(angular_velocity + applied_torque_impulse, p);
  }

  void finish(real_t dt, real_t turn_erp) {
    linear_velocity += delta_v;
    angular_velocity += delta_w;

    target->linear_velocity_ = linear_velocity + applied_force_impulse;
    target->angular_velocity_ = angular_velocity + applied_torque_impulse;

    if(push_applied) {
      integrateTransform(world_transform, push_vel, turn_vel * turn_erp, dt,
                         target->transform);
    }
  }
};
}
}
#endif