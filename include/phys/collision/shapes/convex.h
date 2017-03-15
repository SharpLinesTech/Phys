#ifndef PHYS_COLLISION_SHAPES_CONVEX_H
#define PHYS_COLLISION_SHAPES_CONVEX_H

#include "phys/collision/shape.h"

namespace phys {
namespace shapes {
template <typename CFG>
class Convex : public Shape<CFG> {
 public:
  PHYS_SHAPE_DEF(CONVEX_SHAPE, Shape<CFG>);

  using vec3_t = typename CFG::vec3_t;

  virtual vec3_t getSupportingVertex(vec3_t const& direction) const = 0;
};
}
}

#endif