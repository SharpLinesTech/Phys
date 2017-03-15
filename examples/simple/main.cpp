#include "phys/phys.h"

#include <iostream>

int main(int argc, const char* argv[]) {

  // This config determines math primitives.
  using Cfg = phys::DefaultConfig;

  // Choose the default broadphase and solver
  using Algos = phys::DefaultAlgos<Cfg>;

  using World = phys::World<Cfg, Algos>;

  using DynamicBody = World::DynamicBody;
  using StaticBody = World::StaticBody;

  // Narrowphase algorithms can be shared between multiple worlds, 
  // so the algorithm factory is an injected dependency.
  phys::col::NarrowphaseFactory<Cfg> narrowphases;
  narrowphases.registerDefaultShapesAndAlgorithms();

  // Insert user shape types and collision algorithms here.
  // ...


  // Prepopulate narrowphases, this is necessary for thread safety.
  narrowphases.prepopulate();

  // Create a world
  World world(2,              // Announce that we expect 2 objects. (This is just a hint)
              &narrowphases); // Provide our narrowphase factory.
  
  // We'll need two shapes:
  // 1. a plane, for the floor.
  // 2. a cube, for the box.
  phys::shapes::AxisAlignedPlane<Cfg> plane_shape(1, 0.0f);
  phys::shapes::Box<Cfg>   box_shape({1.0f, 1.0f, 1.0f});


  // Create the floor.
  StaticBody::Config floor_cfg(&plane_shape);
  world.createBody(floor_cfg);
  

  // Create the box.
  DynamicBody::Config box_cfg(&box_shape);
  box_cfg.initial_transform.setTranslation({ 0.0f, 2.0f, 0.0f });
  auto* box = world.createBody(box_cfg);



  for(int t = 0 ; t < 100; ++t) {

    // Apply gravity to all dynamic objects in the world.
    // Gravity needs to be reapplied every step because accumulated forces
    // are reset.
    Cfg::vec3_t gravity = { 0.0f, -9.81f, 0.0f };
    for (auto b : world.dynamicBodies()) {
      b->applyForce(gravity * b->getMass());
    }

    world.step(1.0f/60.0f);

    std::cout << "Box position at time " << t << " is " << box->getPosition().y << std::endl;
  }
  
  return 0;
}