#include "gtest/gtest.h"

#include "phys/collision/broadphase/axis_sweep.h"
#include "phys/phys.h"

using handle_t =
    typename phys::col::AxisSweepBroadphase<phys::DefaultConfig>::Handle;

TEST(AxisSweepBroadphase, AddFirstObject) {
  using CFG = phys::DefaultConfig;

  phys::col::AxisSweepBroadphase<CFG> bp(10);
  phys::Aabb<CFG> aabb;

  aabb.min_bound = {0.0f, 0.0f, 0.0f};
  aabb.max_bound = {1.0f, 1.0f, 1.0f};

  int count = 0;
  auto on_added = [&count](auto a, auto b) { ++count; };
  auto on_removed = [&count](auto a, auto b) { --count; };

  handle_t handle_1;

  bp.addHandle(&handle_1, aabb, on_added, on_removed);

  for(int i = 0; i < 3; ++i) {
    EXPECT_EQ(4, bp.edges_[i].size());
    EXPECT_EQ(&bp.sentinel_, bp.edges_[i].front().handle);
    EXPECT_EQ(&bp.sentinel_, bp.edges_[i].back().handle);
  }

  // Double check we haven't collided with the sentinel.
  EXPECT_EQ(0, count);
}

TEST(AxisSweepBroadphase, NonCollidingBoxes) {
  using CFG = phys::DefaultConfig;

  phys::col::AxisSweepBroadphase<CFG> bp(10);
  phys::Aabb<CFG> aabb[2];

  aabb[0].min_bound = {0.0f, 0.0f, 0.0f};
  aabb[0].max_bound = {1.0f, 1.0f, 1.0f};

  aabb[1].min_bound = {1.5f, 1.5f, 1.5f};
  aabb[1].max_bound = {2.0f, 2.0f, 2.0f};

  int count = 0;
  auto on_added = [&count](auto a, auto b) { ++count; };
  auto on_removed = [&count](auto a, auto b) { --count; };

  handle_t handle_1;
  handle_t handle_2;

  bp.addHandle(&handle_1, aabb[0], on_added, on_removed);
  bp.addHandle(&handle_2, aabb[1], on_added, on_removed);

  // Double check we haven't collided with the sentinel.
  EXPECT_EQ(0, count);
}

TEST(AxisSweepBroadphase, CollisionAtCreationTime) {
  using CFG = phys::DefaultConfig;

  phys::col::AxisSweepBroadphase<CFG> bp(10);
  phys::Aabb<CFG> aabb[2];

  int count = 0;
  auto on_added = [&count](auto a, auto b) { ++count; };
  auto on_removed = [&count](auto a, auto b) { --count; };

  aabb[0].min_bound = {0.0f, 0.0f, 0.0f};
  aabb[0].max_bound = {1.0f, 1.0f, 1.0f};

  aabb[1].min_bound = {0.0f, 0.0f, 0.0f};
  aabb[1].max_bound = {2.0f, 2.0f, 2.0f};

  handle_t handle_1;
  handle_t handle_2;

  bp.addHandle(&handle_1, aabb[0], on_added, on_removed);
  bp.addHandle(&handle_2, aabb[1], on_added, on_removed);

  EXPECT_EQ(1, count);
}

TEST(AxisSweepBroadphase, 2CollisionsOverlap) {
  using CFG = phys::DefaultConfig;

  phys::col::AxisSweepBroadphase<CFG> bp(10);
  phys::Aabb<CFG> aabb[3];

  int count = 0;
  auto on_added = [&count](auto a, auto b) { ++count; };
  auto on_removed = [&count](auto a, auto b) { --count; };

  aabb[0].min_bound = {0.0f, 0.0f, 0.0f};
  aabb[0].max_bound = {1.0f, 1.0f, 1.0f};

  aabb[1].min_bound = {1.5f, 1.5f, 1.5f};
  aabb[1].max_bound = {2.0f, 2.0f, 2.0f};

  aabb[2].min_bound = {0.0f, 0.0f, 0.0f};
  aabb[2].max_bound = {2.0f, 2.0f, 2.0f};

  handle_t handle_1;
  handle_t handle_2;
  handle_t handle_3;

  bp.addHandle(&handle_1, aabb[0], on_added, on_removed);
  bp.addHandle(&handle_2, aabb[1], on_added, on_removed);
  bp.addHandle(&handle_3, aabb[2], on_added, on_removed);

  EXPECT_EQ(2, count);
}