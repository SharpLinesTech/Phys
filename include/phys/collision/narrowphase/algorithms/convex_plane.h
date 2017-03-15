#ifndef PHYS_COL_NARROWPHASE_ALGORITHM_BOX_PLANE_H
#define PHYS_COL_NARROWPHASE_ALGORITHM_BOX_PLANE_H

#include "phys/collision/narrowphase/narrowphase.h"

namespace phys {
namespace col {
namespace narrow {
template <typename CFG>
class ConvexPlane : public Narrowphase<CFG> {
 public:
  using vec3_t = typename CFG::vec3_t;
  using real_t = typename CFG::real_t;

  enum {
    lhs_type = CONVEX_SHAPE,
    rhs_type = AXIS_ALIGNED_PLANE_SHAPE,
  };

  void process(Collision<CFG>* result) override {
    auto convex_obj = result->objects[0];
    auto plane_obj = result->objects[1];

    // Infinite planes only work with identity transforms.
    assert(plane_obj->transform == Transform<CFG>());

    shapes::Convex<CFG> const* convex_shape =
        static_cast<shapes::Convex<CFG> const*>(convex_obj->shape);
    shapes::AxisAlignedPlane<CFG> const* plane_shape =
        static_cast<shapes::AxisAlignedPlane<CFG> const*>(plane_obj->shape);

    auto normal = plane_shape->getNormal();
    auto plane_d = plane_shape->getDistance();

    Transform<CFG> plane_to_convex = convex_obj->transform.inverse();
    Transform<CFG> convex_to_plane = convex_obj->transform;

    vec3_t normal_in_object_space =
        plane_to_convex.getRotationMatrix() * -normal;
    vec3_t convex_support =
        convex_shape->getSupportingVertex(normal_in_object_space);

    vec3_t vertex_in_plane = convex_to_plane.applyToVec(convex_support);
    real_t distance = dot(normal, vertex_in_plane) - plane_d;

    vec3_t vtx_in_plane_projected = vertex_in_plane - normal * distance;

    if(distance < result->getContactDistance()) {
      addContact(*result, normal, vtx_in_plane_projected, distance);
    }
  }
};
}
}
}

#endif