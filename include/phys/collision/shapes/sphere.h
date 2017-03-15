#ifndef PHYS_COLLISION_SHAPES_SPHERE_H
#define PHYS_COLLISION_SHAPES_SPHERE_H

#include "phys/collision/shapes/convex.h"

namespace phys {
namespace shapes {
template <typename CFG>
class Sphere : public Convex<CFG> {
 public:
  PHYS_SHAPE_DEF(SPHERE_SHAPE, Convex<CFG>);

  using real_t = typename CFG::real_t;
  using vec3_t = typename CFG::vec3_t;

  Sphere(real_t const& radius) : radius_(radius) {}

  void getAabb(Aabb<CFG>* dst, Transform<CFG> const& transform) const override {
    auto const& trans = transform.getTranslation();
    dst->min_bound =
        vec3_t{trans[0] - radius_, trans[1] - radius_, trans[2] - radius_};
    dst->max_bound =
        vec3_t{trans[0] + radius_, trans[1] + radius_, trans[2] + radius_};
  }

  void getInertia(real_t mass, vec3_t& dst) const override {
    auto elem = real_t(0.4) * mass * radius_ * radius_;
    dst = {elem, elem, elem};
  }

  vec3_t getSupportingVertex(vec3_t const& direction) const override {
    return direction * radius_;
  }

  real_t getRadius() const {
    return radius_;
  }

 private:
  real_t radius_;
};
}
}

#endif