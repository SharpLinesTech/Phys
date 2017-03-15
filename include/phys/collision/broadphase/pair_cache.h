#ifndef PHYS_COLLISION_BROADPHASE_PAIR_CACHE_H
#define PHYS_COLLISION_BROADPHASE_PAIR_CACHE_H

#include <unordered_set>
#include "phys/collision/broadphase/broadphase.h"

namespace phys {
namespace col {
template <typename CFG>
struct BroadphasePair {
  phys::col::BroadphaseProxy<CFG>* first_object;
  phys::col::BroadphaseProxy<CFG>* second_object;

  // Each broadphase pair may have narrowphase data associated with it.
};
}
}
namespace std {

// We are going to make a huge assumption:
// This allows us to simply take the bottom 4 bytes of each pointer to
// construct the hash.
template <typename CFG>
struct hash<BroadphasePair> {
  typedef BroadphasePair argument_type;
  typedef std::uint64_t result_type;

  result_type operator()(argument_type const& s) const {
    auto a = s.first_object;
    auto b = s.second_object;
    assert(a < b);
    assert((uintptr_t(a) ^ uintptr_t(b)) < uintptr_t(0xFFFFFFFF));

    uint64_t a_as_uint64_t = uint64_t(a) & 0xFFFFFFFF;
    uint64_t b_as_uint64_t = uint64_t(b) & 0xFFFFFFFF;

    return a_as_uint64_t | (b_as_uint64_t << 32);
  }
};
}

namespace phys {
namespace col {
template <typename CFG>
class BroadphasePairCache {
 public:
  void add(BroadphaseProxy<CFG>* a, BroadphaseProxy<CFG>* b) {
    if(a > b) {
      std::swap(a, b);
    }
    pairs_.emplace(a, b);
  }

  void remove(BroadphaseProxy<CFG>* a, BroadphaseProxy<CFG>* b) {
    if(a > b) {
      std::swap(a, b);
    }
    pairs_.erase(std::make_pair(a, b));
  }

  std::unordered_set<std::pair<BroadphaseProxy<CFG>*, BroadphaseProxy<CFG>*>>
      pairs_;
  size_t size() {
    return pairs_.size();
  }
};
}
}

#endif