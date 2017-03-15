#ifndef PHYS_COL_NARROWPHASE_ALGORITHM_CONVEX_CONVEX_H
#define PHYS_COL_NARROWPHASE_ALGORITHM_CONVEX_CONVEX_H

#include "phys/collision/narrowphase/narrowphase.h"

namespace phys {
namespace col {
namespace narrow {
template <typename CFG>
class ConvexConvex : public Narrowphase<CFG> {
 public:
  enum {
    lhs_type = CONVEX_SHAPE,
    rhs_type = CONVEX_SHAPE,
  };

  void process(Collision<CFG>*) override {
    assert(false);  // Implement me!
  }
};
}
}
}

#endif