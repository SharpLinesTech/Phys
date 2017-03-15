#ifndef PHYS_MISC_STATIC_VECTOR_H
#define PHYS_MISC_STATIC_VECTOR_H

#include <array>
#include <cassert>

namespace phys {

// This static vector behaves similarly to a std::vector, but has a static
// datastore.
template <typename T, std::size_t CAP>
struct StaticVector {
  // This assert is just out of lazyness. There is no reason StaticVector cannot
  // be updated to support non-standardlayouts, There's just no reason to spend
  // time on this until we need it. It's mostly a matter of using
  // std::aligned_storage for the value storage and using in-place
  // construction/destruction.
  static_assert(std::is_standard_layout<T>::value,
                "Static Vector only supports PODs right now.");

  using iterator = typename std::array<T, CAP>::iterator;
  using const_iterator = typename std::array<T, CAP>::const_iterator;

  StaticVector() = default;

  const_iterator begin() const;
  const_iterator end() const;

  iterator begin();
  iterator end();

  template <typename... ARGS_T>
  T& emplace_back(ARGS_T&&... args);

  bool full() const;
  std::size_t size() const;
  void pop_back();
  T& operator[](std::size_t i);

 private:
  std::array<T, CAP> values_;
  std::size_t count_ = 0;
};
}

#include "phys/util_types/impl/static_vector_impl.h"

#endif