#ifndef PHYS_COLLISION_SHAPES_PLANE_H
#define PHYS_COLLISION_SHAPES_PLANE_H

#include "phys/collision/shape.h"

namespace phys {
namespace shapes {

// Infinite planes are only supported if they are alligned with an axis,
// otherwise, they would need an inifite AABB which would be very expensive
// (especially considering that a Y=0 plane is the most common case)
template <typename CFG>
class AxisAlignedPlane : public Shape<CFG> {
 public:
  PHYS_SHAPE_DEF(AXIS_ALIGNED_PLANE_SHAPE, Shape<CFG>);
  using real_t = typename CFG::real_t;
  using vec3_t = typename CFG::vec3_t;

  AxisAlignedPlane(int axis, real_t d) : axis_(axis), d_(d) {}

  void getAabb(Aabb<CFG>* dst, Transform<CFG> const& transform) const override {
    assert(transform == Transform<CFG>());

    auto min_val = std::numeric_limits<real_t>::lowest();
    auto max_val = std::numeric_limits<real_t>::max();

    switch(axis_) {
      case 0:
        dst->min_bound = {d_ - 0.1f, min_val, min_val};
        dst->max_bound = {d_ + 0.1f, max_val, max_val};
        break;
      case 1:
        dst->min_bound = {min_val, d_ - 0.1f, min_val};
        dst->max_bound = {max_val, d_ + 0.1f, max_val};
        break;
      case 2:
        dst->min_bound = {min_val, min_val, d_ - 0.1f};
        dst->max_bound = {max_val, max_val, d_ + 0.1f};
        break;
      default:
        dst->min_bound = {min_val - 0.1f, min_val, min_val};
        dst->max_bound = {max_val + 0.1f, max_val, max_val};
        break;
    }
  }

  vec3_t getNormal() const {
    switch(axis_) {
      case 0:
        return {1.0f, 0.0f, 0.0f};
      case 1:
        return {0.0f, 1.0f, 0.0f};
      case 2:
        return {0.0f, 0.0f, 1.0f};
      default:
        return {0.0f, 1.0f, 0.0f};
        break;
    }
  }

  real_t getDistance() const {
    return d_;
  }

  void getInertia(real_t mass, vec3_t& dst) const override {
    // Why are we calculating the inertia of an infinite plane?
    assert(false);
  }

 private:
  int axis_;
  real_t d_;
};
}
}

#endif