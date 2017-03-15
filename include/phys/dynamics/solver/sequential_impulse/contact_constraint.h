#ifndef PHYS_SEQUENTIAL_INPUT_SOLVER_CONTACT_CONSTRAINT_H
#define PHYS_SEQUENTIAL_INPUT_SOLVER_CONTACT_CONSTRAINT_H

#include <limits>
#include "phys/collision/collision.h"
#include "phys/dynamics/solver/sequential_impulse/config.h"

namespace phys {
namespace seqi_solver {
template <typename CFG>
struct ContactConstraint {
  using real_t = typename CFG::real_t;
  using vec3_t = typename CFG::vec3_t;
  ContactConstraint() {}

  ContactConstraint(seqi_solver::Config<CFG> const& solver_cfg, real_t dt,
                    Contact<CFG>* contact, seqi_solver::Body<CFG>* obj_cache_0,
                    seqi_solver::Body<CFG>* obj_cache_1,
                    vec3_t const& rel_pos_0, vec3_t const& rel_pos_1,
                    real_t relative_vel) {
    real_t dt_inv = real_t(1) / dt;

    contact_ = contact;
    solver_bodies_ = {obj_cache_0, obj_cache_1};

    auto* rb_0 = obj_cache_0->target;
    auto* rb_1 = obj_cache_1->target;

    // calculate cfm and erp
    real_t erp = solver_cfg.erp;
    cfm = solver_cfg.cfm;

    cfm *= dt_inv;

    real_t denom = 0;

    if(rb_0) {
      vec3_t torque_axis = cross(rel_pos_0, contact->ws_normal);
      vec3_t ang_comp = rb_0->inv_inertia_tensor_world_ * torque_axis;

      denom = real_t(1.0) / rb_0->mass_ +
              dot(contact->ws_normal, cross(ang_comp, rel_pos_0));

      normals[0] = contact->ws_normal;
      angular_component[0] = ang_comp;
      relpos_cross_normal[0] = torque_axis;
    } else {
      angular_component[0] = {0, 0, 0};
      relpos_cross_normal[0] = {0, 0, 0};
    }

    if(rb_1) {
      vec3_t torque_axis = cross(rel_pos_1, contact->ws_normal);
      vec3_t ang_comp = rb_1->inv_inertia_tensor_world_ * -torque_axis;

      denom += real_t(1.0) / rb_1->mass_ +
               dot(contact->ws_normal, cross(-ang_comp, rel_pos_1));

      normals[1] = -contact->ws_normal;
      angular_component[1] = ang_comp;
      relpos_cross_normal[1] = -torque_axis;
    } else {
      angular_component[1] = {0, 0, 0};
      relpos_cross_normal[1] = {0, 0, 0};
    }

    jac_diag_ab_inv = real_t(1) / (denom + cfm);

    real_t penetration = contact->distance;
    real_t restitution = contact->total_restitution * -relative_vel;

    if(restitution < 0) {
      restitution = 0;
    }

    vec3_t new_init_vel_0 =
        obj_cache_0->linear_velocity + obj_cache_0->applied_force_impulse;
    vec3_t new_init_vel_1 =
        obj_cache_1->linear_velocity + obj_cache_1->applied_force_impulse;

    vec3_t new_init_ang_vel_0 =
        obj_cache_0->angular_velocity + obj_cache_0->applied_torque_impulse;
    vec3_t new_init_ang_vel_1 =
        obj_cache_1->angular_velocity + obj_cache_1->applied_torque_impulse;

    real_t vel0_dot_n = dot(normals[0], new_init_vel_0) +
                        dot(relpos_cross_normal[0], new_init_ang_vel_0);
    real_t vel1_dot_n = dot(normals[1], new_init_vel_1) +
                        dot(relpos_cross_normal[1], new_init_ang_vel_1);

    real_t positional_error = 0;
    real_t velocity_error = restitution - (vel0_dot_n + vel1_dot_n);

    if(penetration > 0) {
      positional_error = 0;
      velocity_error -= penetration * dt_inv;
    } else {
      positional_error = -penetration * erp * dt_inv;
    }

    real_t p_impulse = positional_error * jac_diag_ab_inv;
    real_t velocity_impulse = velocity_error * jac_diag_ab_inv;

    if(penetration > solver_cfg.split_impulse_penetration_threshold) {
      impulse = p_impulse + velocity_impulse;
      penetration_impulse = real_t(0);
    } else {
      impulse = velocity_impulse;
      penetration_impulse = p_impulse;
    }

    cfm *= jac_diag_ab_inv;
  }

  Contact<CFG>* contact_;
  std::array<seqi_solver::Body<CFG>*, 2> solver_bodies_;

  std::array<vec3_t, 2> normals;
  std::array<vec3_t, 2> angular_component;
  std::array<vec3_t, 2> relpos_cross_normal;

  real_t jac_diag_ab_inv = 0;

  real_t cfm = 0;  // Constraint Force Mixing ratio (makes contacts soft)

  real_t impulse = 0;  // bullet calls this rhs
  real_t applied_impulse = 0;

  // Penetration data
  real_t penetration_impulse = 0;
  real_t applied_push_impulse = 0;

  real_t getLowerLimit() {
    return real_t(0);
  }

  real_t getUpperLimit() {
    return std::numeric_limits<real_t>::max();
  }
};
}
}
#endif