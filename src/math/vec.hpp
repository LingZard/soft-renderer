#pragma once
#include <algorithm>
#include <cassert>
#include <cmath>
#include <functional>
#include <initializer_list>
#include <numeric>
#include <ostream>
#include <stdexcept>

#include "../types.hpp"

// ===========================
// VecBase<T, N, Derived>
// ===========================
template <typename T, u32 N, typename Derived>
struct VecBase {
  T data_[N]{};

  using Self = VecBase<T, N, Derived>;

  VecBase() = default;
  VecBase(const T& value) { std::fill(data_, data_ + N, value); }
  VecBase(std::initializer_list<T> list) {
    assert(list.size() == N);
    std::copy(list.begin(), list.end(), data_);
  }

  static Derived zero() {
    Derived result;
    std::fill(result.data_, result.data_ + N, T{0});
    return result;
  }

  static Derived ones() {
    Derived result;
    std::fill(result.data_, result.data_ + N, T{1});
    return result;
  }

  T* begin() { return data_; };
  T* end() { return data_ + N; };
  const T* begin() const { return data_; };
  const T* end() const { return data_ + N; };

  T& operator[](u32 index) {
    assert(index < N);
    return data_[index];
  }
  const T& operator[](u32 index) const {
    assert(index < N);
    return data_[index];
  }

  Derived operator+(const Derived& other) const {
    Derived result;
    std::transform(data_, data_ + N, other.data_, result.data_, std::plus<>());
    return result;
  }

  Derived operator-(const Derived& other) const {
    Derived result;
    std::transform(data_, data_ + N, other.data_, result.data_, std::minus<>());
    return result;
  }

  Derived operator*(const T& scalar) const {
    Derived result;
    std::transform(data_, data_ + N, result.data_,
                   [scalar](const T& val) { return val * scalar; });
    return result;
  }

  Derived& operator+=(const Derived& other) {
    std::transform(data_, data_ + N, other.data_, data_, std::plus<>());
    return static_cast<Derived&>(*this);
  }

  Derived& operator-=(const Derived& other) {
    std::transform(data_, data_ + N, other.data_, data_, std::minus<>());
    return static_cast<Derived&>(*this);
  }

  Derived& operator*=(const T& scalar) {
    std::transform(data_, data_ + N, data_,
                   [scalar](const T& val) { return val * scalar; });
    return static_cast<Derived&>(*this);
  }

  bool operator==(const Self& other) const {
    return this == &other || std::equal(data_, data_ + N, other.data_);
  }

  T dot(const Derived& other) const {
    return std::inner_product(data_, data_ + N, other.data_, T{0});
  }

  T norm() const { return std::sqrt(dot(static_cast<const Derived&>(*this))); }

  void normalize() {
    T n = norm();
    if (n == 0) {
      throw std::runtime_error("Cannot normalize a zero vector");
    }
    for (u32 i = 0; i < N; ++i) {
      data_[i] /= n;
    }
  }

  Derived normalized() const {
    Derived result = static_cast<const Derived&>(*this);
    result.normalize();
    return result;
  }

  constexpr u32 length() const { return N; }

  friend std::ostream& operator<<(std::ostream& os, const Self& vec) {
    os << "[";
    for (u32 i = 0; i < N; ++i) {
      os << vec.data_[i];
      if (i < N - 1) {
        os << ", ";
      }
    }
    os << "]";
    return os;
  }
};

// ===========================
// VecBase<T, N, Derived>
// ===========================
template <typename T, u32 N>
struct Vec : public VecBase<T, N, Vec<T, N>> {
  using Base = VecBase<T, N, Vec<T, N>>;
  using Base::Base;
};

// ===========================
// Specialization for Vec<T, 2>
// ===========================
template <typename T>
struct Vec<T, 2> : public VecBase<T, 2, Vec<T, 2>> {
  using Base = VecBase<T, 2, Vec<T, 2>>;
  using Base::Base;

  T& x() { return this->data_[0]; }
  T& y() { return this->data_[1]; }
  const T& x() const { return this->data_[0]; }
  const T& y() const { return this->data_[1]; }
};

// ===========================
// Specialization for Vec<T, 3>
// ===========================
template <typename T>
struct Vec<T, 3> : public VecBase<T, 3, Vec<T, 3>> {
  using Base = VecBase<T, 3, Vec<T, 3>>;
  using Base::Base;

  T& x() { return this->data_[0]; }
  T& y() { return this->data_[1]; }
  T& z() { return this->data_[2]; }
  const T& x() const { return this->data_[0]; }
  const T& y() const { return this->data_[1]; }
  const T& z() const { return this->data_[2]; }

  Vec cross(const Vec& other) const {
    return Vec{
        this->data_[1] * other.data_[2] - this->data_[2] * other.data_[1],
        this->data_[2] * other.data_[0] - this->data_[0] * other.data_[2],
        this->data_[0] * other.data_[1] - this->data_[1] * other.data_[0]};
  }
};

// ===========================
// Specialization for Vec<T, 4>
// ===========================
template <typename T>
struct Vec<T, 4> : public VecBase<T, 4, Vec<T, 4>> {
  using Base = VecBase<T, 4, Vec<T, 4>>;
  using Base::Base;

  T& x() { return this->data_[0]; }
  T& y() { return this->data_[1]; }
  T& z() { return this->data_[2]; }
  T& w() { return this->data_[3]; }
  const T& x() const { return this->data_[0]; }
  const T& y() const { return this->data_[1]; }
  const T& z() const { return this->data_[2]; }
  const T& w() const { return this->data_[3]; }
};

// ===========================
// frequently used types
// ===========================
using Vec2f = Vec<float, 2>;
using Vec3f = Vec<float, 3>;
using Vec4f = Vec<float, 4>;
using Vec2i = Vec<int, 2>;
using Vec3i = Vec<int, 3>;
using Vec4i = Vec<int, 4>;
using Vec2u = Vec<u32, 2>;
using Vec3u = Vec<u32, 3>;
using Vec4u = Vec<u32, 4>;
