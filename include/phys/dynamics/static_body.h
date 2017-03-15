#ifndef PHYS_LIB_DYNAMICS_STATIC_BODY_H
#define PHYS_LIB_DYNAMICS_STATIC_BODY_H

#include "phys/dynamics/body.h"
namespace phys {

template <typename CFG, typename ALGO>
class StaticBody : public Body<CFG, ALGO> {
 public:
  struct Config : public Body<CFG, ALGO>::Config {
    Config(Shape<CFG>* shp) : Body<CFG, ALGO>::Config(shp) {}
  };

  StaticBody(Config const& cfg) : Body<CFG, ALGO>(cfg) {
    collision_info_.owner_type_ = col::Object<CFG>::STATIC_OBJECT;
  }

 private:
};
}

#endif