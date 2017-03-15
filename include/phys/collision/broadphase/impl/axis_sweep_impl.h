#ifndef PHYS_COLLISION_BROADPHASE_AXIS_SWEEP_IMPL_H
#define PHYS_COLLISION_BROADPHASE_AXIS_SWEEP_IMPL_H

#include "phys/collision/broadphase/axis_sweep.h"

namespace phys {
namespace col {

template <typename CFG>
AxisSweepBroadphase<CFG>::AxisSweepBroadphase(uint32_t object_count_hint) {
  auto expected_edge_per_axis = (object_count_hint + 1) * 2;

  auto min_val = std::numeric_limits<real_t>::lowest();
  auto max_val = std::numeric_limits<real_t>::max();
  for(int i = 0; i < 3; ++i) {
    sentinel_.min_edges_[i] = 0;
    sentinel_.max_edges_[i] = 1;

    edges_[i].reserve(expected_edge_per_axis);

    // Insert the sentinel
    edges_[i].emplace_back(Edge_{min_val, &sentinel_, false});
    edges_[i].emplace_back(Edge_{max_val, &sentinel_, true});
  }
}

template <typename CFG>
template <typename PAIR_ADDED_CB, typename PAIR_REMOVED_CB>
void AxisSweepBroadphase<CFG>::addHandle(Handle* new_handle,
                                         Aabb<CFG> const& aabb,
                                         PAIR_ADDED_CB on_added,
                                         PAIR_REMOVED_CB on_removed) {
  for(int i = 0; i < 3; ++i) {
    // Remove the sentinel
    edges_[i].pop_back();

    // Insert the new object at the end
    new_handle->min_edges_[i] = uint32_t(edges_[i].size());
    edges_[i].emplace_back(Edge_{aabb.min_bound[i], new_handle, false});

    new_handle->max_edges_[i] = uint32_t(edges_[i].size());
    edges_[i].emplace_back(Edge_{aabb.max_bound[i], new_handle, true});

    // replace the sentinel
    edges_[i].emplace_back(
        Edge_{std::numeric_limits<real_t>::max(), &sentinel_, true});

    sentinel_.max_edges_[i] = uint32_t(edges_[i].size() - 1);
  }

  sortMinDown_(0, new_handle->min_edges_[0], [](Handle*) {});
  sortMaxDown_(0, new_handle->max_edges_[0], [](Handle*) {});
  sortMinDown_(1, new_handle->min_edges_[1], [](Handle*) {});
  sortMaxDown_(1, new_handle->max_edges_[1], [](Handle*) {});

  // It "might" be worth creating a smaller temporary cache of collisions
  // instead of commiting to the central cache as we go along...
  sortMinDown_(2, new_handle->min_edges_[2],
               [new_handle, on_added](Handle* b) { on_added(new_handle, b); });
  sortMaxDown_(
      2, new_handle->max_edges_[2],
      [new_handle, on_removed](Handle* b) { on_removed(new_handle, b); });
}

template <typename CFG>
template <typename PAIR_ADDED_CB, typename PAIR_REMOVED_CB>
void AxisSweepBroadphase<CFG>::updateHandle(Handle* hndl,
                                            Aabb<CFG> const& new_aabb,
                                            PAIR_ADDED_CB added,
                                            PAIR_REMOVED_CB removed) {
  auto on_added = [hndl, added](Handle* b) { added(hndl, b); };

  auto on_removed = [hndl, removed](Handle* b) { removed(hndl, b); };

  for(int axis = 0; axis < 3; ++axis) {
    auto min_edge = hndl->min_edges_[axis];
    auto max_edge = hndl->max_edges_[axis];

    auto dmin = new_aabb.min_bound[axis] - edges_[axis][min_edge].position;
    auto dmax = new_aabb.max_bound[axis] - edges_[axis][max_edge].position;

    edges_[axis][min_edge].position = new_aabb.min_bound[axis];
    edges_[axis][max_edge].position = new_aabb.max_bound[axis];

    // expand (only adds overlaps)
    if(dmin < 0)
      sortMinDown_(axis, min_edge, on_added);

    if(dmax > 0)
      sortMaxUp_(axis, max_edge, on_added);

    // shrink (only removes overlaps)
    if(dmin > 0)
      sortMinUp_(axis, min_edge, on_removed);

    if(dmax < 0)
      sortMaxDown_(axis, max_edge, on_removed);
  }
}

template <typename CFG>
void AxisSweepBroadphase<CFG>::removeHandle(Handle* proxy) {
  // TO BE IMPLEMENTED.
}

template <typename CFG>
template <typename ADD_CB>
void AxisSweepBroadphase<CFG>::sortMinDown_(int axis, uint32_t edge_id,
                                            ADD_CB cb) {
  Edge_* edge = &edges_[axis][edge_id];
  Edge_* prev_edge = edge - 1;
  Handle* handle = edge->handle;

  while(edge->position < prev_edge->position) {
    Handle* prev_handle = prev_edge->handle;

    if(prev_edge->is_max) {
      const int axis1 =
          (1 << axis) &
          3;  // equivalent to: (axis + 1) % 3, but faster (tested)
      const int axis2 = (1 << axis1) & 3;

      if(testOverlap2D_(handle, prev_handle, axis1, axis2)) {
        cb(prev_handle);
      }
      prev_handle->max_edges_[axis]++;
    } else {
      prev_handle->min_edges_[axis]++;
    }

    handle->min_edges_[axis]--;

    std::swap(*edge, *prev_edge);

    // The sentinel value will force us out of the loop if necessary.
    edge--;
    prev_edge--;
  }
}

template <typename CFG>
template <typename REM_CB>
void AxisSweepBroadphase<CFG>::sortMinUp_(int axis, uint32_t edge_id,
                                          REM_CB cb) {
  Edge_* edge = &edges_[axis][edge_id];
  Edge_* next_edge = edge + 1;
  Handle* handle = edge->handle;

  while(edge->position > next_edge->position) {
    Handle* next_handle = next_edge->handle;

    if(next_edge->is_max) {
      const int axis1 =
          (1 << axis) &
          3;  // equivalent to: (axis + 1) % 3, but faster (tested)
      const int axis2 = (1 << axis1) & 3;

      if(testOverlap2D_(handle, next_handle, axis1, axis2)) {
        cb(next_handle);
      }
      next_handle->max_edges_[axis]--;
    } else {
      next_handle->min_edges_[axis]--;
    }

    handle->min_edges_[axis]++;

    std::swap(*edge, *next_edge);

    // The sentinel value will force us out of the loop if necessary.
    edge--;
    next_edge--;
  }
}

template <typename CFG>
template <typename ADD_CB>
void AxisSweepBroadphase<CFG>::sortMaxUp_(int axis, uint32_t edge_id,
                                          ADD_CB cb) {
  Edge_* edge = &edges_[axis][edge_id];
  Edge_* next_edge = edge + 1;
  Handle* handle = edge->handle;

  while(edge->position > next_edge->position) {
    Handle* next_handle = next_edge->handle;

    if(!next_edge->is_max) {
      const int axis1 =
          (1 << axis) &
          3;  // equivalent to: (axis + 1) % 3, but faster (tested)
      const int axis2 = (1 << axis1) & 3;

      if(testOverlap2D_(handle, next_handle, axis1, axis2)) {
        cb(next_handle);
      }
      next_handle->min_edges_[axis]--;
    } else {
      next_handle->max_edges_[axis]--;
    }

    handle->max_edges_[axis]++;

    std::swap(*edge, *next_edge);

    // The sentinel value will force us out of the loop if necessary.
    edge++;
    next_edge++;
  }
}

template <typename CFG>
template <typename REM_CB>
void AxisSweepBroadphase<CFG>::sortMaxDown_(int axis, uint32_t edge_id,
                                            REM_CB cb) {
  Edge_* edge = &edges_[axis][edge_id];
  Edge_* prev_edge = edge - 1;
  Handle* handle = edge->handle;

  while(edge->position < prev_edge->position) {
    Handle* prev_handle = prev_edge->handle;

    // N.B. if cb is a no-op, this entire section gets optimized away.
    if(!prev_edge->is_max) {
      const int axis1 =
          (1 << axis) &
          3;  // equivalent to: (axis + 1) % 3, but faster (tested)
      const int axis2 = (1 << axis1) & 3;

      if(testOverlap2D_(handle, prev_handle, axis1, axis2)) {
        cb(prev_handle);
      }
      prev_handle->min_edges_[axis]++;
    } else {
      prev_handle->max_edges_[axis]++;
    }

    handle->max_edges_[axis]--;

    std::swap(*edge, *prev_edge);

    // The sentinel value will force us out of the loop if necessary.
    edge--;
    prev_edge--;
  }
}

template <typename CFG>
bool AxisSweepBroadphase<CFG>::testOverlap2D_(Handle* handle_1,
                                              Handle* handle_2, int axis_1,
                                              int axis_2) {
  /*if (0 == (handle_1->collision_mask & handle_2->collision_mask)) {
    return false;
  }*/

  if(handle_1->max_edges_[axis_1] < handle_2->min_edges_[axis_1] ||
     handle_2->max_edges_[axis_1] < handle_1->min_edges_[axis_1] ||
     handle_1->max_edges_[axis_2] < handle_2->min_edges_[axis_2] ||
     handle_2->max_edges_[axis_2] < handle_1->min_edges_[axis_2]) {
    return false;
  }
  return true;
}
}
}

#endif