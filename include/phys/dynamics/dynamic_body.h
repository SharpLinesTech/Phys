#ifndef PHYS_LIB_DYNAMICS_RIGID_BODY_H
#define PHYS_LIB_DYNAMICS_RIGID_BODY_H

#include "phys/dynamics/body.h"
namespace phys {

template <typename CFG>
struct DynamicBodyData {
  using real_t = typename CFG::real_t;
  using vec3_t = typename CFG::vec3_t;
  using mat3x3_t = typename CFG::mat3x3_t;

  Transform<CFG>* transform;  // UGHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH

  vec3_t net_force_ = {0, 0, 0};
  vec3_t net_torque_ = {0, 0, 0};

  real_t mass_ = 1;
  vec3_t linear_velocity_ = {0, 0, 0};
  vec3_t angular_velocity_ = {0, 0, 0};

  vec3_t inv_inertia_local_;
  mat3x3_t inv_inertia_tensor_world_;
};

template <typename CFG, typename ALGO>
class DynamicBody : public Body<CFG, ALGO> {
 public:
  using real_t = typename CFG::real_t;
  using vec3_t = typename CFG::vec3_t;
  using mat3x3_t = typename CFG::mat3x3_t;

  struct Config : public Body<CFG, ALGO>::Config {
    Config(Shape<CFG>* shp) : Body<CFG, ALGO>::Config(shp) {}
    real_t mass = 1.0;
  };

  DynamicBody(Config const& cfg) : Body<CFG, ALGO>(cfg) {
    collision_info_.owner_type_ = col::Object<CFG>::DYNAMIC_OBJECT;

    dynamics_data_.mass_ = cfg.mass;
    vec3_t inertia;
    cfg.shape->getInertia(cfg.mass, inertia);

    dynamics_data_.transform = &collision_info_.transform;

    dynamics_data_.inv_inertia_local_[0] = real_t(1.0) / inertia[0];
    dynamics_data_.inv_inertia_local_[1] = real_t(1.0) / inertia[1];
    dynamics_data_.inv_inertia_local_[2] = real_t(1.0) / inertia[2];

    updateInertia();
  }

  vec3_t getLinearVelocity() const {
    return dynamics_data_.linear_velocity_;
  }

  void setLinearVelocity(vec3_t const& vel) {
    dynamics_data_.linear_velocity_ = vel;
  }

  vec3_t getAngularVelocity() const {
    return dynamics_data_.angular_velocity_;
  }

  void setAngularVelocity(vec3_t const& vel) {
    dynamics_data_.angular_velocity_ = vel;
  }

  real_t getMass() const {
    return dynamics_data_.mass_;
  }
  real_t getInvMass() const {
    return real_t(1) / dynamics_data_.mass_;
  }

  void applyForce(vec3_t const& force) {
    dynamics_data_.net_force_ += force;
  }

  void clearForces() {
    dynamics_data_.net_force_ = {0, 0, 0};
    dynamics_data_.net_torque_ = {0, 0, 0};
  }

  void updateInertia() {
    auto const& rot = collision_info_.transform.getRotationMatrix();
    dynamics_data_.inv_inertia_tensor_world_ = {
        rot[0] * dynamics_data_.inv_inertia_local_[0],
        rot[1] * dynamics_data_.inv_inertia_local_[1],
        rot[2] * dynamics_data_.inv_inertia_local_[2]};

    dynamics_data_.inv_inertia_tensor_world_ *= transpose(rot);
  }

  mat3x3_t const& getInvInertiaTensorWorld() const {
    return inv_inertia_tensor_world_;
  }

  static DynamicBody* fromCollisionObject(col::Object<CFG>* col_object) {
    if(col_object->owner_type_ == col::Object<CFG>::DYNAMIC_OBJECT) {
      return static_cast<DynamicBody*>(col_object->owner_);
    }
    return nullptr;
  }

  DynamicBodyData<CFG> dynamics_data_;

  // Currently assigned simulation island.
  uint32_t island_id_;

  // Assigned solver internal state
  uint32_t solver_id_;

  // Index in the dynamic world table
  std::size_t world_index_;
};
}

#endif