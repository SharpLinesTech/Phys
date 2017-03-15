#ifndef PHYS_TYPES_AABB_IMPL_H
#define PHYS_TYPES_AABB_IMPL_H

namespace phys {

template <typename CFG>
Aabb<CFG>::Aabb() {}

template <typename CFG>
Aabb<CFG>::Aabb(vec3_t const& half_extent, Transform<CFG> const& transform) {
  auto const& trans = transform.getTranslation();
  auto const& rot = transform.getRotationMatrix();
  vec3_t rotated_extent = abs(rot * half_extent);

  min_bound = trans - rotated_extent;
  max_bound = trans + rotated_extent;
}
}

#endif