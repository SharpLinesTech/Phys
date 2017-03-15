#ifndef PHYS_COLLISION_SHAPE_H
#define PHYS_COLLISION_SHAPE_H

#include "phys/math_types/aabb.h"
#include "phys/math_types/transform.h"

namespace phys {

enum ShapeType {
  UNKNOWN_SHAPE = -1,
  CONVEX_SHAPE,

  BOX_SHAPE,
  SPHERE_SHAPE,
  AXIS_ALIGNED_PLANE_SHAPE,
};

#define PHYS_SHAPE_DEF(type, parent_class)                                  \
  enum { shape_type = type, parent_shape_type = parent_class::shape_type }; \
  int getShapeType() const override {                                       \
    return type;                                                            \
  }  \
//end of macro
template <typename CFG>
class Shape {
 public:
  using vec3_t = typename CFG::vec3_t;
  using real_t = typename CFG::real_t;

  enum { shape_type = UNKNOWN_SHAPE };

  virtual void getAabb(Aabb<CFG>* dst,
                       Transform<CFG> const& transform) const = 0;
  virtual int getShapeType() const = 0;

  virtual void getInertia(real_t mass, vec3_t& dst) const = 0;
};
}

#endif