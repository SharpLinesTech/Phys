#ifndef PHYS_TYPES_TRANSFORM_H
#define PHYS_TYPES_TRANSFORM_H

namespace phys {

template <typename CFG>
struct Transform {
  using vec3_t = typename CFG::vec3_t;
  using mat3x3_t = typename CFG::mat3x3_t;
  using mat4x4_t = typename CFG::mat4x4_t;
  using quat_t = typename CFG::quat_t;

  Transform();
  Transform(mat3x3_t const& rot, vec3_t const& trans);

  vec3_t const& getTranslation() const;
  void setTranslation(vec3_t const& t);

  mat3x3_t const& getRotationMatrix() const;
  void setRotation(quat_t const& from_quat);

  mat4x4_t getMatrix() const;

  Transform inverse() const;

  bool operator==(Transform const& rhs) const;

  vec3_t applyToVec(vec3_t const& vec) const;
  vec3_t applyInverse(vec3_t const& vec) const;

 private:
  mat3x3_t rotation_;
  vec3_t translation_;
};

template <typename CFG>
inline void integrateTransform(Transform<CFG> const& init_trans,
                               typename CFG::vec3_t const& linvel,
                               typename CFG::vec3_t const& angvel,
                               typename CFG::real_t const& dt,
                               Transform<CFG>* dst);
}

#include "phys/math_types/impl/transform_impl.h"

#endif