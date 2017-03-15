#ifndef PHYS_MISC_ARRAY_VIEW_IMPL_H
#define PHYS_MISC_ARRAY_VIEW_IMPL_H

#include <vector>

namespace phys {

template <typename T>
ArrayView<T>::ArrayView(T* start, std::size_t count)
    : start_(start), count_(count) {}

template <typename T>
ArrayView<T>::ArrayView(typename std::vector<T>::iterator b,
                        typename std::vector<T>::iterator e)
    : start_(b == e ? nullptr : &(*b)), count_(e - b) {}

template <typename T>
T* ArrayView<T>::begin() const {
  return start_;
}

template <typename T>
T* ArrayView<T>::end() const {
  return start_ + count_;
}

template <typename T>
T& ArrayView<T>::operator[](std::size_t i) const {
  return start_[i];
}

template <typename T>
std::size_t ArrayView<T>::size() const {
  return count_;
}
}

#endif