#ifndef PHYS_SEQUENTIAL_INPUT_SOLVER_SOLVER_H
#define PHYS_SEQUENTIAL_INPUT_SOLVER_SOLVER_H

#include "phys/dynamics/solver/sequential_impulse/body.h"
#include "phys/dynamics/solver/sequential_impulse/config.h"
#include "phys/dynamics/solver/sequential_impulse/contact_constraint.h"
#include "phys/util_types/array_view.h"

namespace phys {
namespace seqi_solver {
template <typename CFG>
class Solver {
 public:
  using real_t = typename CFG::real_t;
  using vec3_t = typename CFG::vec3_t;

  Solver(Config<CFG> const& cfg = Config<CFG>()) : config_(cfg) {}

  template <typename ALGO>
  void solve(ArrayView<DynamicBody<CFG, ALGO>*> objects,
             ArrayView<Collision<CFG>*> collisions, real_t dt) {
    dt_ = dt;
    setup_(objects, collisions);

    resolvePenetrations_();

    for(int i = 0; i < config_.iterations; ++i) {
      auto residual = solveIteration_();

      if(residual <= config_.residual_threshold) {
        break;
      }
    }

    finish_();
  }

  template <typename ALGO>
  void setup_(ArrayView<DynamicBody<CFG, ALGO>*> objects,
              ArrayView<Collision<CFG>*> collisions) {
    bodies_.resize(0);
    contacts_.resize(0);

    bodies_.reserve(objects.size());

    for(auto obj : objects) {
      obj->solver_id_ = uint32_t(bodies_.size());
      bodies_.emplace_back(&obj->dynamics_data_, dt_);
    }
    auto col_count = collisions.size();

    for(auto col : collisions) {
      addCollision_<ALGO>(col);
    }
  }

  void resolvePenetrations_() {
    for(int iteration = 0; iteration < config_.penetration_iterations;
        ++iteration) {
      real_t residual = 0.0f;
      for(auto& contact : contacts_) {
        residual += solvePenetration_(contact);
      }

      if(residual <= config_.residual_threshold) {
        break;
      }
    }
  }

  real_t solveIteration_() {
    real_t residual = real_t(0);

    // Solve generic constraints

    // Solve contacts
    for(auto& contact : contacts_) {
      auto contact_residual = solveContact_(contact);
      residual += contact_residual * contact_residual;
    }

    // Solve friction

    return residual;
  }

  void finish_() {
    for(auto& body : bodies_) {
      auto* object = body.target;

      body.finish(dt_, config_.split_impulse_turn_erp);
    }
  }

  real_t solvePenetration_(seqi_solver::ContactConstraint<CFG>& c) {
    real_t d_impulse = 0;

    if(c.penetration_impulse != real_t(0)) {
      auto* body_0 = c.solver_bodies_[0];
      auto* body_1 = c.solver_bodies_[1];

      d_impulse = c.penetration_impulse - c.applied_push_impulse * c.cfm;

      real_t dv_0_dot_n = dot(c.normals[0], body_0->push_vel) +
                          dot(c.relpos_cross_normal[0], body_0->turn_vel);
      real_t dv_1_dot_n = dot(c.normals[1], body_1->push_vel) +
                          dot(c.relpos_cross_normal[1], body_1->turn_vel);

      d_impulse -= dv_0_dot_n * c.jac_diag_ab_inv;
      d_impulse -= dv_1_dot_n * c.jac_diag_ab_inv;
      auto new_impulse = c.applied_push_impulse + d_impulse;

      if(new_impulse < c.getLowerLimit()) {
        d_impulse = c.getLowerLimit() - c.applied_push_impulse;
        new_impulse = c.getLowerLimit();
      }

      c.applied_push_impulse = new_impulse;

      body_0->applyPushImpulse(c.normals[0] * body_0->inv_mass,
                               c.angular_component[0], d_impulse);
      body_1->applyPushImpulse(c.normals[1] * body_1->inv_mass,
                               c.angular_component[1], d_impulse);
    }

    return d_impulse;
  }

  real_t solveContact_(seqi_solver::ContactConstraint<CFG>& c) {
    real_t d_impulse = c.impulse - c.applied_impulse * c.cfm;

    auto* body_0 = c.solver_bodies_[0];
    auto* body_1 = c.solver_bodies_[1];

    real_t dv_0_dot_n = dot(c.normals[0], body_0->delta_v) +
                        dot(c.relpos_cross_normal[0], body_0->delta_w);
    real_t dv_1_dot_n = dot(c.normals[1], body_1->delta_v) +
                        dot(c.relpos_cross_normal[1], body_1->delta_w);

    d_impulse -= dv_0_dot_n * c.jac_diag_ab_inv;
    d_impulse -= dv_1_dot_n * c.jac_diag_ab_inv;

    auto new_impulse = c.applied_impulse + d_impulse;

    if(new_impulse < c.getLowerLimit()) {
      d_impulse = c.getLowerLimit() - c.applied_impulse;
      new_impulse = c.getLowerLimit();
    }

    c.applied_impulse = new_impulse;

    body_0->applyImpulse(c.normals[0] * body_0->inv_mass,
                         c.angular_component[0], d_impulse);
    body_1->applyImpulse(c.normals[1] * body_1->inv_mass,
                         c.angular_component[1], d_impulse);

    return d_impulse;
  }

 private:
  real_t dt_;

  Config<CFG> config_;
  std::vector<seqi_solver::Body<CFG>> bodies_;
  std::vector<seqi_solver::ContactConstraint<CFG>> contacts_;

  // Static body shared by all static objects.
  seqi_solver::Body<CFG> staticBody;

  template <typename ALGO>
  void addCollision_(Collision<CFG>* col) {
    auto dyn_obj_0 =
        DynamicBody<CFG, ALGO>::fromCollisionObject(col->objects[0]);
    auto dyn_obj_1 =
        DynamicBody<CFG, ALGO>::fromCollisionObject(col->objects[1]);

    seqi_solver::Body<CFG>* solver_body_0 =
        dyn_obj_0 ? &bodies_[dyn_obj_0->solver_id_] : &staticBody;
    seqi_solver::Body<CFG>* solver_body_1 =
        dyn_obj_1 ? &bodies_[dyn_obj_1->solver_id_] : &staticBody;

    // No fuzzy-float check needed, infinite mass should always be a
    // hard-assigned 0. Hitting this should imply the existence of a
    // non-Kinematic infinite-mass dynamic object, which should not be a thing.
    assert(solver_body_0->inv_mass != real_t(0) ||
           solver_body_1->inv_mass != real_t(0));

    // HERE: bullet implements contactProcessingThreshold check
    for(auto& contact : col->points) {
      // Precalculate a few things so that we don't duplicate work in the
      // various contact/friction constructors.
      vec3_t rel_pos_0 =
          contact.ws_position[0] - col->objects[0]->transform.getTranslation();
      vec3_t rel_pos_1 =
          contact.ws_position[1] - col->objects[1]->transform.getTranslation();

      vec3_t vel_0 = solver_body_0->getRelativeVelocity(rel_pos_0);
      vec3_t vel_1 = solver_body_1->getRelativeVelocity(rel_pos_1);

      vec3_t vel = vel_0 - vel_1;
      real_t relative_vel = dot(contact.ws_normal, vel);

      contacts_.emplace_back(config_, dt_, &contact, solver_body_0,
                             solver_body_1, rel_pos_0, rel_pos_1, relative_vel);
    }
  }
};
}
}
#endif