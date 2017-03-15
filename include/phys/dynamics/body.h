#ifndef PHYS_LIB_DYNAMICS_BODY_H
#define PHYS_LIB_DYNAMICS_BODY_H

#include "phys/collision/collision_world.h"
#include "phys/collision/shape.h"
#include "phys/math_types/aabb.h"
#include "phys/math_types/transform.h"

namespace phys {
template <typename CFG, typename ALGO>
struct Body {
 public:
  using Broadphase = typename ALGO::Broadphase;
  using CollisionInfo = typename col::BP_Object<CFG, Broadphase>;

  struct Config {
    Config(Shape<CFG>* shp) : shape(shp) {}
    Shape<CFG>* shape = nullptr;
    Transform<CFG> initial_transform;
  };

  Body(Config const& config) {
    collision_info_.shape = config.shape;
    collision_info_.transform = config.initial_transform;
    collision_info_.owner_ = this;
  }

  auto getPosition() const {
    return collision_info_.transform.getTranslation();
  }

  auto const& getTransform() const {
    return collision_info_.transform;
  }

  auto& getTransform() {
    return collision_info_.transform;
  }

  void getAabb(Aabb<CFG>* dst) {
    collision_info_.getAabb(dst);
  }

  CollisionInfo collision_info_;
};
}

#endif