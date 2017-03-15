#ifndef PHYS_LIB_DYNAMICS_WORLD_IMPL_H
#define PHYS_LIB_DYNAMICS_WORLD_IMPL_H

namespace phys {

template <typename CFG, typename ALGO>
World<CFG, ALGO>::World(unsigned int object_count_hint,
                        col::NarrowphaseFactory<CFG>* np_factory)
    : collision_world_(object_count_hint, np_factory) {}

template <typename CFG, typename ALGO>
void World<CFG, ALGO>::step(real_t dt) {
  // Find intersections
  detectCollisions_();

  island_manager.buildAndVisitIslands(
      collision_world_.collisions(), dynamic_bodies_,
      [dt, this](auto objects, auto collisions) {

        solver_.solve(objects, collisions, dt);
      });

  // Integrate transforms.
  for(auto b : dynamic_bodies_) {
    Transform<CFG> new_trans;
    integrateTransform(b->collision_info_.transform, b->getLinearVelocity(),
                       b->getAngularVelocity(), dt, &new_trans);

    b->collision_info_.transform = new_trans;

    b->clearForces();
  }
}

template <typename CFG, typename ALGO>
void World<CFG, ALGO>::detectCollisions_() {
  // Update the broadphase AABB pf all bodies that may have moved.
  for(auto b : dynamic_bodies_) {
    if(b->collision_info_.isActive()) {
      Aabb<CFG> aabb;
      b->getAabb(&aabb);

      collision_world_.update(&b->collision_info_);
    }
  }

  collision_world_.updateNarrowphase();
}

template <typename CFG, typename ALGO>
typename World<CFG, ALGO>::StaticBody* World<CFG, ALGO>::createBody(
    typename World<CFG, ALGO>::StaticBody::Config const& cfg) {
  auto result = new StaticBody(cfg);

  // No need to keep track of static bodies, they'll be handled entirely by
  // collision detection.

  bodyCreated_(result);
  return result;
}

template <typename CFG, typename ALGO>
typename World<CFG, ALGO>::DynamicBody* World<CFG, ALGO>::createBody(
    typename World<CFG, ALGO>::DynamicBody::Config const& cfg) {
  auto result = new DynamicBody(cfg);

  result->world_index_ = dynamic_bodies_.size();
  dynamic_bodies_.emplace_back(result);

  bodyCreated_(result);
  return result;
}

template <typename CFG, typename ALGO>
void World<CFG, ALGO>::deleteBody(StaticBody* body) {
  bodyDestroyed_(body);
  delete body;
}

template <typename CFG, typename ALGO>
void World<CFG, ALGO>::deleteBody(DynamicBody* body) {
  bodyDestroyed_(body);

  auto index = body->world_index_;
  assert(dynamic_bodies_[index] == body);
  dynamic_bodies_[index] = dynamic_bodies_.back();
  dynamic_bodies_[index]->world_index_ = index;
  dynamic_bodies_.pop_back();
  delete body;
}

template <typename CFG, typename ALGO>
void World<CFG, ALGO>::bodyCreated_(Body* body) {
  Aabb<CFG> aabb;
  body->getAabb(&aabb);

  collision_world_.add(&body->collision_info_);
}

template <typename CFG, typename ALGO>
void World<CFG, ALGO>::bodyDestroyed_(Body* body) {
  collision_world_.remove(&body->collision_info_);
}

template <typename CFG, typename ALGO>
std::vector<typename World<CFG, ALGO>::DynamicBody*>&
World<CFG, ALGO>::dynamicBodies() {
  return dynamic_bodies_;
}
}

#endif