#ifndef PHYS_COLLISION_BROADPHASE_AXIS_SWEEP_H
#define PHYS_COLLISION_BROADPHASE_AXIS_SWEEP_H

#include <deque>
#include <list>
#include <vector>
#include "phys/math_types/aabb.h"

namespace phys {
namespace col {
template <typename CFG>
class AxisSweepBroadphase {
 public:
  using real_t = typename CFG::real_t;
  using vec3_t = typename CFG::vec3_t;

  struct Handle {
    uint32_t min_edges_[3];
    uint32_t max_edges_[3];
  };

  // Args:
  //   object_count_hint: number of objects we are expecting to handle.
  AxisSweepBroadphase(uint32_t object_count_hint);

  //  Args:
  //   1: The new handle to register
  //   2: the initial aabb
  //   3: callback to invoke when a pair is added
  //   4: callback to invoke when a pair is removed
  template <typename PAIR_ADDED_CB, typename PAIR_REMOVED_CB>
  void addHandle(Handle*, Aabb<CFG> const&, PAIR_ADDED_CB on_added,
                 PAIR_REMOVED_CB on_removed);

  //  Args:
  //   1: The handle to update
  //   2: the updated aabb
  //   3: callback to invoke when a pair is added
  //   4: callback to invoke when a pair is removed
  template <typename PAIR_ADDED_CB, typename PAIR_REMOVED_CB>
  void updateHandle(Handle*, Aabb<CFG> const&, PAIR_ADDED_CB, PAIR_REMOVED_CB);

  // Args:
  //  1: The handle to remove
  // N.B. It's implicitely understood that every pair involving the handle is
  // being removed.
  void removeHandle(Handle*);

  // private:

  // The very first and last entries for each axis will refer to this handle,
  // this will allow us to avoid bounds checking.
  Handle sentinel_;

  // DEVNOTE: Since edges get swapped around a LOT during the sorting process,
  // it might be worth storing the handle pointer in a separate table
  // and store an index instead. (and use bitwise tricks to encode is_max)
  // If we can crunch this down to 64 bits, the benefits might be substantial.
  struct Edge_ {
    real_t position;
    Handle* handle;
    bool is_max;
  };

  std::vector<Edge_> edges_[3];

  // Expands
  template <typename ADD_CB>
  void sortMinDown_(int axis, uint32_t edge, ADD_CB);

  template <typename ADD_CB>
  void sortMaxUp_(int axis, uint32_t edge, ADD_CB);

  // Shrinks
  template <typename REM_CB>
  void sortMinUp_(int axis, uint32_t edge, REM_CB);

  template <typename REM_CB>
  void sortMaxDown_(int axis, uint32_t edge, REM_CB);

  bool testOverlap2D_(Handle* handle_1, Handle* handle_2, int axis_1,
                      int axis_2);
};
}
}

#include "phys/collision/broadphase/impl/axis_sweep_impl.h"

#endif