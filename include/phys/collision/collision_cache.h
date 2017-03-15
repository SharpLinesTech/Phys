#ifndef PHYS_COLLISION_OBJECT_PAIR_H
#define PHYS_COLLISION_OBJECT_PAIR_H

#include <cassert>
#include <unordered_map>
#include "phys/collision/collision.h"
#include "phys/collision/narrowphase/narrowphase.h"

namespace phys {
namespace col {
// Holds information about the collision between two objects
template <typename CFG>
struct CollisionCacheEntry {
  CollisionCacheEntry(Object<CFG>* a, Object<CFG>* b) {
    // This allows the narrowphase algorithms to make assumptions
    // related to the types of the objects.
    if(a->shape->getShapeType() > b->shape->getShapeType()) {
      std::swap(a, b);
    }

    collision.objects = {a, b};
  }

  // The collision itself.
  Collision<CFG> collision;

  // The narrowphase algorithm assigned to this collision.
  NarrowPhasePtr<CFG> narrowphase_;
};

template <typename CFG>
std::uint64_t getCollisionCacheKey(Object<CFG>* a, Object<CFG>* b) {
  if(a > b) {
    std::swap(a, b);
  }
  // This is mildly dangerous, but not THAT much, we assume that every object
  // within a given world lives within the same 4GB memory range.
  assert((uintptr_t(a) ^ uintptr_t(b)) < uintptr_t(0xFFFFFFFF));
  uint64_t a_as_uint64_t = uint64_t(a) & 0xFFFFFFFF;
  uint64_t b_as_uint64_t = uint64_t(b) & 0xFFFFFFFF;

  return a_as_uint64_t | (b_as_uint64_t << 32);
}

template <typename CFG>
struct CollisionCache {
  void add(Object<CFG>* a, Object<CFG>* b) {
    auto key = getCollisionCacheKey(a, b);

    // We don't instantiate the narrowphase right away as the object will often
    // be immedaitely removed
    cache_.emplace(key, CollisionCacheEntry<CFG>{a, b});
  }

  void remove(Object<CFG>* a, Object<CFG>* b) {
    auto key = getCollisionCacheKey(a, b);
    cache_.erase(key);
  }

  std::unordered_map<int64_t, CollisionCacheEntry<CFG>> cache_;
};
}
}

#endif