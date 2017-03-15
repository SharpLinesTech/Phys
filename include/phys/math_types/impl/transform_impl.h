#ifndef PHYS_TYPES_TRANSFORM_IMPL_H
#define PHYS_TYPES_TRANSFORM_IMPL_H

namespace phys {
template <typename CFG>
Transform<CFG>::Transform()
    : rotation_(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
      translation_(0.0, 0.0, 0.0) {}

template <typename CFG>
Transform<CFG>::Transform(mat3x3_t const& rot, vec3_t const& trans)
    : rotation_(rot), translation_(trans) {}

template <typename CFG>
typename CFG::vec3_t const& Transform<CFG>::getTranslation() const {
  return translation_;
}

template <typename CFG>
void Transform<CFG>::setTranslation(vec3_t const& t) {
  translation_ = t;
}

template <typename CFG>
typename CFG::mat3x3_t const& Transform<CFG>::getRotationMatrix() const {
  return rotation_;
}

template <typename CFG>
typename CFG::mat4x4_t Transform<CFG>::getMatrix() const {
  mat4x4_t result = rotation_;

  result[3][0] = translation_[0];
  result[3][1] = translation_[1];
  result[3][2] = translation_[2];
  return result;
}

template <typename CFG>
void Transform<CFG>::setRotation(quat_t const& from_quat) {
  rotation_ = mat3_cast(from_quat);
}

template <typename CFG>
Transform<CFG> Transform<CFG>::inverse() const {
  mat3x3_t inv_rot = transpose(rotation_);
  return Transform(inv_rot, inv_rot * -translation_);
}

template <typename CFG>
bool Transform<CFG>::operator==(Transform const& rhs) const {
  return rotation_ == rhs.rotation_ && translation_ == rhs.translation_;
}

template <typename CFG>
typename CFG::vec3_t Transform<CFG>::applyToVec(vec3_t const& vec) const {
  return rotation_ * vec + translation_;
}

template <typename CFG>
typename CFG::vec3_t Transform<CFG>::applyInverse(vec3_t const& vec) const {
  return transpose(rotation_) * (vec - translation_);
}

template <typename CFG>
void integrateTransform(Transform<CFG> const& init_trans,
                        typename CFG::vec3_t const& linvel,
                        typename CFG::vec3_t const& angvel,
                        typename CFG::real_t const& dt, Transform<CFG>* dst) {
  using vec3_t = typename CFG::vec3_t;
  using real_t = typename CFG::real_t;
  using quat_t = typename CFG::quat_t;

  dst->setTranslation(init_trans.getTranslation() + linvel * dt);

  vec3_t axis;
  real_t angle = length(angvel);

  if(angle < real_t(0.001)) {
    // use Taylor's expansions of sync function
    axis = angvel *
           (real_t(0.5) * dt -
            (dt * dt * dt) * (real_t(0.020833333333)) * angle * angle);
  } else {
    // sync(fAngle) = sin(c*fAngle)/t
    axis = angvel * (sin(real_t(0.5) * angle * dt) / angle);
  }

  quat_t dorn(cos(angle * dt * real_t(0.5)), axis[0], axis[1], axis[2]);
  quat_t orn0 = init_trans.getRotationMatrix();

  quat_t predictedOrn = normalize(dorn * orn0);

  dst->setRotation(predictedOrn);
}
}

#endif