#ifndef PHYS_TYPES_AABB_H
#define PHYS_TYPES_AABB_H

#include "phys/math_types/transform.h"

namespace phys {

template <typename CFG>
struct Aabb {
  using vec3_t = typename CFG::vec3_t;

  Aabb();
  Aabb(vec3_t const& half_extent, Transform<CFG> const& transform);

  vec3_t min_bound;
  vec3_t max_bound;
};
}

#include "phys/math_types/impl/aabb_impl.h"

#endif