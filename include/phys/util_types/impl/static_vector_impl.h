#ifndef PHYS_MISC_STATIC_VECTOR_IMPL_H
#define PHYS_MISC_STATIC_VECTOR_IMPL_H

#include <array>
#include <cassert>

namespace phys {

template <typename T, std::size_t CAP>
typename StaticVector<T, CAP>::const_iterator StaticVector<T, CAP>::begin()
    const {
  return values_.begin();
}

template <typename T, std::size_t CAP>
typename StaticVector<T, CAP>::const_iterator StaticVector<T, CAP>::end()
    const {
  return values_.end();
}

template <typename T, std::size_t CAP>
typename StaticVector<T, CAP>::iterator StaticVector<T, CAP>::begin() {
  return values_.begin();
}

template <typename T, std::size_t CAP>
typename StaticVector<T, CAP>::iterator StaticVector<T, CAP>::end() {
  return values_.begin() + count_;
}

template <typename T, std::size_t CAP>
template <typename... ARGS_T>
T& StaticVector<T, CAP>::emplace_back(ARGS_T&&... args) {
  assert(!full());
  return values_[count_++] = T(std::forward<ARGS_T>(args)...);
}

template <typename T, std::size_t CAP>
bool StaticVector<T, CAP>::full() const {
  return count_ == CAP;
}

template <typename T, std::size_t CAP>
std::size_t StaticVector<T, CAP>::size() const {
  return count_;
}

template <typename T, std::size_t CAP>
void StaticVector<T, CAP>::pop_back() {
  --count_;
}

template <typename T, std::size_t CAP>
T& StaticVector<T, CAP>::operator[](std::size_t i) {
  return values_[i];
}
}

#endif