#ifndef PHYS_LIB_DYNAMICS_SIMULATION_ISLAND_MANAGER_H
#define PHYS_LIB_DYNAMICS_SIMULATION_ISLAND_MANAGER_H

#include "phys/collision/collision_world.h"
#include "phys/util_types/array_view.h"

#include <algorithm>

namespace phys {

struct IslandMapping {
  int object_id;
  int island_id;
};

template <typename CFG>
class SimulationIslandManager {
 public:
  // Building and visiting must be a single function
  // because adding or removing objects to the world between
  // these two steps is undefined.
  template <typename COLLISIONS, typename BODY_T, typename VISIT_T>
  void buildAndVisitIslands(COLLISIONS& collisions_set,
                            std::vector<BODY_T*>& bodies, VISIT_T visitor) {
    auto obj_count = bodies.size();
    // we assume obj_count > 0 when we disptach islands
    if(obj_count == 0) {
      return;
    }

    buildIslands_(collisions_set, bodies);

    // Sort objects in the collision world by island.
    std::sort(bodies.begin(), bodies.end(), [](auto* lhs, auto* rhs) {
      return lhs->island_id_ < rhs->island_id_;
    });

    // Sort the collisions and constraints. By definition, they should only ever
    // belong to a single island.
    sorted_collisions_.resize(0);
    for(auto& collision : collisions_set) {
      auto* col = &collision.second.collision;

      uint32_t island_id = 0xFFFFFFFF;
      auto dyn_obj = BODY_T::fromCollisionObject(col->objects[0]);

      if(!dyn_obj) {
        dyn_obj = BODY_T::fromCollisionObject(col->objects[1]);
      }

      if(dyn_obj) {
        col->island_id_ = dyn_obj->island_id_;
      } else {
        col->island_id_ = 0xFFFFFFFF;
      }

      sorted_collisions_.push_back(col);
    }

    std::sort(sorted_collisions_.begin(), sorted_collisions_.end(),
              [](auto* a, auto* b) { return a->island_id_ < b->island_id_; });

    // Bootstrap using the first object.
    auto island_start = bodies.begin();
    (*island_start)->world_index_ = 0;

    auto col_start = sorted_collisions_.begin();

    unsigned int current_island = (*island_start)->island_id_;
    unsigned int i = 1;
    auto obj = island_start + 1;
    for(; obj != bodies.end(); ++obj) {
      // Since we sorted that list, we must update the reverse index.
      (*obj)->world_index_ = i++;

      auto island = (*obj)->island_id_;
      if(island != current_island) {
        // Visit the island.
        auto col_end = col_start;
        while(col_end != sorted_collisions_.end() &&
              (*col_end)->island_id_ == current_island) {
          ++col_end;
        }

        ArrayView<BODY_T*> objects(island_start, obj);
        ArrayView<Collision<CFG>*> collisions(col_start, col_end);

        visitor(objects, collisions);

        // reset the process
        island_start = obj;
        col_start = col_end;
        current_island = island;
      }
    }

    // Visit the final island.
    auto col_end = col_start;
    while(col_end != sorted_collisions_.end() &&
          (*col_end)->island_id_ == current_island) {
      ++col_end;
    }

    ArrayView<BODY_T*> objs(island_start, obj);
    ArrayView<Collision<CFG>*> collisions(col_start, col_end);

    visitor(objs, collisions);
  }

  template <typename VISITOR_T>
  void visitIslands(VISITOR_T visit) {
    int island_start = 0;
    while(island_start < island_mapping.size()) {
      int island_id = island_mapping[island_start].island_id;
      int island_end = island_start + 1;
      while(island_end < island_mapping.size() &&
            island_mapping[island_start].island_id == island_id) {
        ++island_end;
      }

      int island_size = island_start - island_end;

      ArrayView<>

          island_start = island_end;
    }
  }

 private:
  std::vector<IslandMapping> island_mapping;
  std::vector<Collision<CFG>*> sorted_collisions_;

  template <typename COLLISIONS, typename BODY_T>
  void buildIslands_(COLLISIONS& collisions, std::vector<BODY_T*>& bodies) {
    uint32_t obj_count = uint32_t(bodies.size());

    island_mapping.resize(obj_count);
    for(uint32_t i = 0; i < obj_count; ++i) {
      bodies[i]->island_id_ = i;

      island_mapping[i].object_id = i;
      island_mapping[i].island_id = i;
    }

    // Use collisions to join islands.
    for(auto& collision_entry : collisions) {
      auto& col = collision_entry.second.collision;

      auto obj_a = col.objects[0];
      auto obj_b = col.objects[1];

      auto dyn_obj_a = BODY_T::fromCollisionObject(obj_a);
      auto dyn_obj_b = BODY_T::fromCollisionObject(obj_b);
      if(dyn_obj_a && dyn_obj_b) {
        // TODO: identify body from the collision object
        joinIslands_(dyn_obj_a->island_id_, dyn_obj_b->island_id_);
      }
    }

    // Assign final island ids to objects.
    for(uint32_t i = 0; i < obj_count; ++i) {
      bodies[i]->island_id_ = findIsland_(i);
    }
  }

  int findIsland_(int obj_id) {
    int parent = island_mapping[obj_id].island_id;
    if(obj_id == parent || island_mapping[parent].island_id == parent) {
      return parent;
    }

    auto result = findIsland_(parent);

    // Cache the result for faster future lookup.
    island_mapping[obj_id].island_id = result;
    return result;
  }

  void joinIslands_(int obj_a, int obj_b) {
    auto island_a = findIsland_(obj_a);
    auto island_b = findIsland_(obj_b);

    if(island_a != island_b) {
      // we don;t bother balancing the tree because it'll get flattenedd upon
      // lookup.
      island_mapping[island_a].island_id = island_b;
    }
  }
};
}

#endif
