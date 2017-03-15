#ifndef PHYS_COL_NARROWPHASE_ALGORITHM_SPHERE_SPHERE_H
#define PHYS_COL_NARROWPHASE_ALGORITHM_SPHERE_SPHERE_H

#include "phys/collision/narrowphase/narrowphase.h"

namespace phys {
namespace col {
namespace narrow {
template <typename CFG>
class SphereSphere : public Narrowphase<CFG> {
 public:
  enum {
    lhs_type = SPHERE_SHAPE,
    rhs_type = SPHERE_SHAPE,
  };

  void process(Collision<CFG>* result) override {
    auto sphere_a = result->objects[0];
    auto sphere_b = result->objects[1];

    shapes::Sphere<CFG> const* sphere_a_shape =
        static_cast<shapes::Sphere<CFG> const*>(sphere_a->shape);
    shapes::Sphere<CFG> const* sphere_b_shape =
        static_cast<shapes::Sphere<CFG> const*>(sphere_b->shape);

    auto contact_dst =
        sphere_a_shape->getRadius() + sphere_a_shape->getRadius();

    auto delta_p = sphere_a->transform.getTranslation() -
                   sphere_b->transform.getTranslation();
    auto dst_sq = dot(delta_p, delta_p);
    auto len = sqrt(dst_sq);

    auto dist = len - contact_dst;
    if(dist < result->getContactDistance()) {
      typename CFG::vec3_t normal = {0, 0, 0};
      if(len > 0) {
        normal = delta_p / len;
      }

      auto col_point_on_b = sphere_b->transform.getTranslation() +
                            normal * sphere_a_shape->getRadius();
      addContact(*result, normal, col_point_on_b, dist);
    }
  }
};
}
}
}

#endif