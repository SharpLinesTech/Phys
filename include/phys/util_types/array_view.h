#ifndef PHYS_MISC_ARRAY_VIEW_H
#define PHYS_MISC_ARRAY_VIEW_H

#include <vector>

namespace phys {
template <typename T>
struct ArrayView {
  ArrayView(T* start, std::size_t count);

  ArrayView(typename std::vector<T>::iterator b,
            typename std::vector<T>::iterator e);

  T* begin() const;
  T* end() const;

  T& operator[](std::size_t i) const;

  std::size_t size() const;

 private:
  T* start_;
  std::size_t count_;
};
}

#include "phys/util_types/impl/array_view_impl.h"

#endif