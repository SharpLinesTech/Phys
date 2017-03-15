#ifndef PHYS_COL_NARROWPHASE_ALGORITHM_BOX_BOX_H
#define PHYS_COL_NARROWPHASE_ALGORITHM_BOX_BOX_H

#include "phys/collision/narrowphase/narrowphase.h"

namespace phys {
namespace col {
namespace narrow {
template <typename CFG>
class BoxBox : public Narrowphase<CFG> {
 public:
  using vec3_t = typename CFG::vec3_t;
  using real_t = typename CFG::real_t;

  enum {
    lhs_type = BOX_SHAPE,
    rhs_type = BOX_SHAPE,
  };

  void process(Collision<CFG>* result) override {
    assert(false);  // Implement me!
  }
};
}
}
}

#endif