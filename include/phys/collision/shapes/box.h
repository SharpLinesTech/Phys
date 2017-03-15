#ifndef PHYS_COLLISION_SHAPES_BOX_H
#define PHYS_COLLISION_SHAPES_BOX_H

#include "phys/collision/shapes/convex.h"

namespace phys {
namespace shapes {
template <typename CFG>
class Box : public Convex<CFG> {
  vec3_t half_extent_;

 public:
  PHYS_SHAPE_DEF(BOX_SHAPE, Convex<CFG>);

  using real_t = typename CFG::real_t;
  using vec3_t = typename CFG::vec3_t;

  Box(vec3_t const& half_extent) : half_extent_(half_extent) {}

  void getAabb(Aabb<CFG>* dst, Transform<CFG> const& transform) const override {
    *dst = Aabb<CFG>(half_extent_, transform);
  }

  vec3_t getSupportingVertex(vec3_t const& direction) const override {
    auto x = direction[0] >= 0 ? half_extent_[0] : -half_extent_[0];
    auto y = direction[1] >= 0 ? half_extent_[1] : -half_extent_[1];
    auto z = direction[2] >= 0 ? half_extent_[2] : -half_extent_[2];

    return {x, y, z};
  }

  void getInertia(real_t mass, vec3_t& dst) const override {
    auto size = half_extent_ * real_t(2);
    auto size_sq = size * size;

    dst[0] = mass / real_t(12) * (size_sq[1] + size_sq[2]);
    dst[1] = mass / real_t(12) * (size_sq[0] + size_sq[2]);
    dst[2] = mass / real_t(12) * (size_sq[0] + size_sq[1]);
  }

  vec3_t getHalfExtent() const {
    return half_extent_;
  }
};
}
}

#endif