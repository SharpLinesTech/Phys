#ifndef PHYS_COL_NARROWPHASE_H
#define PHYS_COL_NARROWPHASE_H

#include <cassert>
#include <functional>
#include <map>
#include <memory>
#include "phys/collision/collision_object.h"
#include "phys/collision/shape.h"

namespace phys {
namespace col {

template <typename CFG>
Contact<CFG>* leastValuablePoint(Collision<CFG>& collision,
                                 typename CFG::vec3_t const& local_a,
                                 typename CFG::real_t dist) {
  using point_ite = typename Collision<CFG>::points_container::iterator;
  using real_t = typename CFG::real_t;
  using vec3_t = typename CFG::vec3_t;

  auto& points = collision.points;

  // Make sure we do not get rid of the point with deepest penetration.
  point_ite max_depth_point = points.end();
  real_t max_depth = dist;
  for(auto point = points.begin(); point != points.end(); ++point) {
    if(point->distance < max_depth) {
      max_depth_point = point;
      max_depth = point->distance;
    }
  }

  point_ite candidate;
  real_t candidate_value = std::numeric_limits<real_t>::max();
  for(auto point = points.begin(); point != points.end(); ++point) {
    if(point == max_depth_point) {
      continue;
    }

    // The value of a point is determined by how far it is from all other
    // points Bullet does this by calculating the area of the coverage
    // generated by the other points. But it assumes max_points=4. This should
    // also be faster and effectively as good (whough may have some degenerate
    // cases).
    vec3_t d = local_a - point->os_position[0];
    real_t value = dot(d, d);
    for(auto alt_point = points.begin(); alt_point != points.end();
        ++alt_point) {
      d = point->os_position[0] - alt_point->os_position[0];
      value += dot(d, d);
    }

    if(value < candidate_value) {
      candidate_value = value;
      candidate = point;
    }
  }

  return &*candidate;
}

template <typename CFG>
Contact<CFG>* getPoint(Collision<CFG>& collision,
                       typename CFG::vec3_t const& local_a,
                       typename CFG::real_t dist,
                       typename CFG::real_t threshold_sq) {
  // First, try to find an existing point that's close enough to qualify as
  // "equivalent".
  auto closest_dist_sq = threshold_sq;
  for(auto point = collision.points.begin(); point != collision.points.end();
      ++point) {
    auto d_a = local_a - point->os_position[0];
    auto dist_sq = dot(d_a, d_a);
    if(dist_sq < closest_dist_sq) {
      return &*point;
    }
  }
  // Next, if we have room for a new one.
  if(!collision.points.full()) {
    return &collision.points.emplace_back();
  }

  // recycle something.
  return leastValuablePoint(collision, local_a, dist);
}

template <typename CFG>
void addContact(Collision<CFG>& collision, typename CFG::vec3_t const& normal,
                typename CFG::vec3_t point_on_b,
                typename CFG::real_t distance) {
  using vec3_t = typename CFG::vec3_t;

  auto a = collision.objects[0];
  auto b = collision.objects[1];

  vec3_t point_on_a = point_on_b + normal * distance;
  vec3_t local_a = a->transform.applyInverse(point_on_a);

  auto dst =
      getPoint(collision, local_a, distance, collision.getContactDistanceSq());

  dst->ws_position[0] = point_on_a;
  dst->ws_position[1] = point_on_b;

  dst->os_position[0] = local_a;
  dst->os_position[1] = b->transform.applyInverse(point_on_b);
  dst->ws_normal = normal;
  dst->distance = distance;
}

// By default, narrowphase instances are shared, so it cannot have per-contact
// member variables. Do not attempt to go around this by using a pair->value map
// variable. Use StatefullNarrowphase instead.
template <typename CFG>
class Narrowphase {
 public:
  virtual ~Narrowphase() {}
  virtual void process(Collision<CFG>*) = 0;

  // If a Narrowphase returns true here, it will be instantiated for each
  // relevant collision pair, otherwise a single instance will be shared.
  virtual bool statefull() const {
    return false;
  }

  // This only needs to be overwritten if statefull() returns true.
  virtual std::unique_ptr<Narrowphase<CFG>> clone() const {
    return nullptr;
  }
};

// If a narrowphase needs more statefull data than what is held in a
// CollisionState, then it can inherit from this.
//
template <typename CFG, typename CRTP>
class StatefullNarrowphase : public Narrowphase<CFG> {
 public:
  bool statefull() const override {
    return true;
  }
  virtual std::unique_ptr<Narrowphase<CFG>> clone() const override {
    return std::make_unique<CRTP>();
  }
};

// This could be better implemented as a unique pointer...
template <typename CFG>
struct NarrowPhasePtr {
  NarrowPhasePtr() : narrowphase_(nullptr), cloned_(false) {}

  NarrowPhasePtr(std::unique_ptr<Narrowphase<CFG>> np)
      : narrowphase_(np.release()), cloned_(true) {}

  NarrowPhasePtr(Narrowphase<CFG>* np) : narrowphase_(np), cloned_(false) {}

  ~NarrowPhasePtr() {
    if(narrowphase_ && cloned_) {
      delete narrowphase_;
    }
  }

  NarrowPhasePtr(NarrowPhasePtr const&) = delete;
  NarrowPhasePtr& operator=(NarrowPhasePtr const&) = delete;

  NarrowPhasePtr(NarrowPhasePtr&& rhs)
      : narrowphase_(rhs.narrowphase_), cloned_(rhs.cloned_) {
    rhs.narrowphase_ = nullptr;
  }

  NarrowPhasePtr& operator=(NarrowPhasePtr&& rhs) {
    narrowphase_ = rhs.narrowphase_;
    cloned_ = rhs.cloned_;
    rhs.narrowphase_ = nullptr;
    return *this;
  }

  operator bool() const {
    return narrowphase_ != nullptr;
  }

  Narrowphase<CFG>* operator->() {
    return narrowphase_;
  }

 private:
  Narrowphase<CFG>* narrowphase_;
  bool cloned_;
};

template <typename CFG>
class NarrowphaseFactory {
  using InternalNarrowphasePtr = std::unique_ptr<Narrowphase<CFG>>;

 public:
  NarrowPhasePtr<CFG> getNarrowphase(Shape<CFG>* a, Shape<CFG>* b) {
    int shape_type_a = a->getShapeType();
    int shape_type_b = b->getShapeType();

    if(shape_type_a > shape_type_b) {
      std::swap(a, b);
      std::swap(shape_type_a, shape_type_b);
    }

    auto algo_type = std::make_pair(shape_type_a, shape_type_b);
    auto& algo = narrowphases_[algo_type];

    // Did prepopulate get called?
    assert(algo);

    if(algo->statefull()) {
      return NarrowPhasePtr<CFG>(algo->clone());
    }

    return NarrowPhasePtr<CFG>(algo.get());
  }

  template <typename T>
  void registerShapeType() {
    int type = T::shape_type;
    int parent_type = T::parent_shape_type;

    shape_hierarchy_[type] = parent_type;
  }

  template <typename T>
  void registerAlgorithm(int priority) {
    int lhs_type = T::lhs_type;
    int rhs_type = T::rhs_type;

    assert(lhs_type <= rhs_type);

    auto& dst = factories_[std::make_pair(lhs_type, rhs_type)];

    dst.priority = priority;
    dst.function = std::make_unique<T>;
  }

  void registerDefaultShapesAndAlgorithms();

  // This will populate all algorithm permutations.
  void prepopulate() {
    for(auto lhs_ite = shape_hierarchy_.begin();
        lhs_ite != shape_hierarchy_.end(); ++lhs_ite) {
      auto lhs = lhs_ite->first;

      for(auto rhs_ite = lhs_ite; rhs_ite != shape_hierarchy_.end();
          ++rhs_ite) {
        auto rhs = rhs_ite->first;
        auto found_np = lookupNarrowphaseFactory_(lhs, rhs);
        if(found_np) {
          narrowphases_[std::make_pair(lhs, rhs)] = found_np->function();
        }
      }
    }
  }

 private:
  struct Entry {
    int priority;
    std::function<InternalNarrowphasePtr()> function;
  };

  std::map<int, int> shape_hierarchy_;
  std::map<std::pair<int, int>, Entry> factories_;
  std::map<std::pair<int, int>, InternalNarrowphasePtr> narrowphases_;

  Entry* lookupNarrowphaseFactory_(int a, int b) {
    Entry* result = nullptr;
    int b_orig = b;
    while(b != -1) {
      auto found = factories_.find(std::make_pair(a, b));
      if(found != factories_.end()) {
        if(!result || found->second.priority > result->priority) {
          result = &found->second;
        }
      }

      auto b_parent = shape_hierarchy_.find(b);
      if(b_parent == shape_hierarchy_.end()) {
        b = -1;
      } else {
        b = b_parent->second;
      }
    }

    auto a_parent = shape_hierarchy_.find(a);
    if(a_parent != shape_hierarchy_.end()) {
      Entry* parent_result =
          lookupNarrowphaseFactory_(a_parent->second, b_orig);

      if(!result ||
         (parent_result && parent_result->priority > result->priority)) {
        result = parent_result;
      }
    }
    return result;
  }
};
}
}

#include "phys/collision/shapes/axis_aligned_plane.h"
#include "phys/collision/shapes/box.h"
#include "phys/collision/shapes/convex.h"
#include "phys/collision/shapes/sphere.h"

#include "phys/collision/narrowphase/algorithms/box_box.h"
#include "phys/collision/narrowphase/algorithms/convex_convex.h"
#include "phys/collision/narrowphase/algorithms/convex_plane.h"
#include "phys/collision/narrowphase/algorithms/sphere_sphere.h"

namespace phys {
namespace col {
template <typename CFG>
void NarrowphaseFactory<CFG>::registerDefaultShapesAndAlgorithms() {
  registerShapeType<shapes::Convex<CFG>>();
  registerShapeType<shapes::Box<CFG>>();
  registerShapeType<shapes::Sphere<CFG>>();
  registerShapeType<shapes::AxisAlignedPlane<CFG>>();

  registerAlgorithm<narrow::ConvexConvex<CFG>>(0);
  registerAlgorithm<narrow::ConvexPlane<CFG>>(0);

  registerAlgorithm<narrow::BoxBox<CFG>>(1);
  registerAlgorithm<narrow::SphereSphere<CFG>>(1);
}
}
}

#endif