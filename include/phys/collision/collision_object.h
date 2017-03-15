#ifndef PHYS_COLLISION_OBJECT_H
#define PHYS_COLLISION_OBJECT_H

#include "phys/collision/broadphase/axis_sweep.h"
#include "phys/collision/shape.h"
#include "phys/math_types/transform.h"
namespace phys {

namespace col {
// An object pair represents two objects that we suspect might be colliding
template <typename CFG>
struct Object {
  enum OwnerType {
    NO_OWNER,
    DYNAMIC_OBJECT,
    STATIC_OBJECT,
    KINEMATIC_OBJECT,
  };

  // user-relevant fields.
  Shape<CFG>* shape;
  Transform<CFG> transform;

  // index of the object in the collision world.
  uint32_t world_index_;

  OwnerType owner_type_ = NO_OWNER;
  void* owner_ = nullptr;

  bool isActive() const {
    return true;
  }

  bool acceptsForces() const {
    return owner_type == DYNAMIC_OBJECT;
  }

  void getAabb(Aabb<CFG>* dst) {
    shape->getAabb(dst, transform);
  }
};

template <typename CFG, typename BROADPHASE_T>
struct BP_Object : public Object<CFG> {
  using bp_handle_t = typename BROADPHASE_T::Handle;

  // Internal info.
  bp_handle_t bp_handle_;

  static Object* getFromBpHandle(bp_handle_t* hndl) {
    auto offset = std::intptr_t(&((BP_Object*)(nullptr))->bp_handle_);

    return reinterpret_cast<BP_Object*>(reinterpret_cast<char*>(hndl) - offset);
  }
};
}
}

#endif