#ifndef PHYS_LIB_DYNAMICS_WORLD_H
#define PHYS_LIB_DYNAMICS_WORLD_H

#include "phys/dynamics/dynamic_body.h"
#include "phys/dynamics/simulation_island_manager.h"
#include "phys/dynamics/static_body.h"

#include <vector>

namespace phys {
template <typename CFG, typename ALGO>
struct World {
  using real_t = typename CFG::real_t;
  using vec3_t = typename CFG::vec3_t;

  using Broadphase = typename ALGO::Broadphase;
  using Solver = typename ALGO::Solver;

  using Body = Body<CFG, ALGO>;
  using StaticBody = StaticBody<CFG, ALGO>;
  using DynamicBody = DynamicBody<CFG, ALGO>;

  World(unsigned int object_count_hint,
        col::NarrowphaseFactory<CFG>* np_factory);
  void step(real_t);

  StaticBody* createBody(typename StaticBody::Config const&);
  DynamicBody* createBody(typename DynamicBody::Config const&);

  void deleteBody(StaticBody*);
  void deleteBody(DynamicBody*);

  std::vector<DynamicBody*>& dynamicBodies();

 private:
  std::vector<DynamicBody*> dynamic_bodies_;

  BP_CollisionWorld<CFG, Broadphase> collision_world_;
  SimulationIslandManager<CFG> island_manager;

  void detectCollisions_();

  void bodyCreated_(Body*);
  void bodyDestroyed_(Body*);

  Solver solver_;
};
}

#include "phys/dynamics/impl/dynamics_world_impl.h"

#endif