#ifndef PHYS_COLLISION_COLLISION_WORLD_H
#define PHYS_COLLISION_COLLISION_WORLD_H

#include <cstdint>
#include "phys/collision/broadphase/axis_sweep.h"
#include "phys/collision/collision_cache.h"
#include "phys/collision/narrowphase/narrowphase.h"

namespace phys {

// A collision world is a persistent structure that maintains and updates
// collisions between "objects". It's expected that there will be some form of
// temporal coherency, so the structure is built with that in mind.
template <typename CFG>
struct CollisionWorld {
  using Object = col::Object<CFG>;

  CollisionWorld(col::NarrowphaseFactory<CFG>* np_factory)
      : narrowphase_factory_(np_factory) {}

  using collision_mask_t = typename CFG::collision_mask_t;

  std::unordered_map<int64_t, col::CollisionCacheEntry<CFG>>& collisions() {
    return collisions_cache_.cache_;
  }

  std::vector<Object*>& objects() {
    return objects_;
  }

  std::vector<Object*>& isolatableObjects() {
    return isolatable_objects_;
  }

  void updateNarrowphase() {
    for(auto& pair : collisions()) {
      auto& entry = pair.second;

      if(!entry.narrowphase_) {
        // find a narrowphase for this pair.
        pair.second.narrowphase_ = narrowphase_factory_->getNarrowphase(
            entry.collision.objects[0]->shape,
            entry.collision.objects[1]->shape);
      }

      // update existing contacts before adding any new ones.
      entry.collision.refresh();

      pair.second.narrowphase_->process(&entry.collision);
    }
  }

  std::vector<Object*> objects_;

  // These are the objects that are candidates for simulation island generation.
  std::vector<Object*> isolatable_objects_;

  col::NarrowphaseFactory<CFG>* narrowphase_factory_;
  col::CollisionCache<CFG> collisions_cache_;
};

template <typename CFG, typename BROADPHASE_T>
struct BP_CollisionWorld : public CollisionWorld<CFG> {
  using Broadphase = BROADPHASE_T;
  using bp_handle_t = typename Broadphase::Handle;
  using BP_Object = col::BP_Object<CFG, BROADPHASE_T>;

  BP_CollisionWorld(uint32_t object_count_hint,
                    col::NarrowphaseFactory<CFG>* np_factory)
      : CollisionWorld<CFG>(np_factory), broadphase_(object_count_hint) {}

  void add(BP_Object* obj) {
    obj->world_index_ = uint32_t(objects_.size());
    objects_.push_back(obj);

    auto on_added = [this](bp_handle_t* a, bp_handle_t* b) {
      Object* obj_a = BP_Object::getFromBpHandle(a);
      Object* obj_b = BP_Object::getFromBpHandle(b);
      collisions_cache_.add(obj_a, obj_b);
    };

    auto on_removed = [this](bp_handle_t* a, bp_handle_t* b) {
      Object* obj_a = BP_Object::getFromBpHandle(a);
      Object* obj_b = BP_Object::getFromBpHandle(b);
      collisions_cache_.remove(obj_a, obj_b);
    };

    Aabb<CFG> init_aabb;
    obj->getAabb(&init_aabb);
    broadphase_.addHandle(&obj->bp_handle_, init_aabb, on_added, on_removed);
  }

  void remove(BP_Object* obj) {
    // remove the object from our list.
    auto index = obj->world_index_;
    objects_[index] = objects_.back();
    objects_[index]->world_index_ = index objects_.pop_back();

    collisions_cache_.removeAll(obj);
    broadphase_.removeHandle(&obj->bp_handle_);
  }

  void update(BP_Object* obj) {
    auto on_added = [this](bp_handle_t* a, bp_handle_t* b) {
      Object* obj_a = BP_Object::getFromBpHandle(a);
      Object* obj_b = BP_Object::getFromBpHandle(b);
      collisions_cache_.add(obj_a, obj_b);
    };

    auto on_removed = [this](bp_handle_t* a, bp_handle_t* b) {
      Object* obj_a = BP_Object::getFromBpHandle(a);
      Object* obj_b = BP_Object::getFromBpHandle(b);
      collisions_cache_.remove(obj_a, obj_b);
    };

    Aabb<CFG> new_aabb;
    obj->getAabb(&new_aabb);
    broadphase_.updateHandle(&obj->bp_handle_, new_aabb, on_added, on_removed);
  }

  Broadphase broadphase_;
};
}

#endif