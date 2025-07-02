#pragma once
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <type_traits>

namespace soft_renderer {
namespace math {

// ===========================
// VecBase
// ===========================
template <typename T, uint32_t N, typename Derived>
struct VecBase {
 protected:
  using Self = VecBase<T, N, Derived>;
  T data_[N]{};

 public:
  // 1. Types and Constants
  using Scalar = T;

  static constexpr T epsilon() {
    if constexpr (std::is_floating_point_v<T>) {
      return std::numeric_limits<T>::epsilon() * 100;
    } else {
      return T{0};
    }
  }

  // 2. Constructors
  VecBase() = default;
  VecBase(const T& value) { std::fill(data_, data_ + N, value); }
  VecBase(std::initializer_list<T> list) {
    assert(list.size() == N);
    std::copy(list.begin(), list.end(), data_);
  }

  // 3. Static Factory Methods
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

  // 4. Accessors
  T& operator[](uint32_t index) {
    assert(index < N);
    return data_[index];
  }
  const T& operator[](uint32_t index) const {
    assert(index < N);
    return data_[index];
  }

  T* begin() { return data_; };
  T* end() { return data_ + N; };
  const T* begin() const { return data_; };
  const T* end() const { return data_ + N; };

  // 5. Arithmetic Operators (Vector-Vector)
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

  Derived operator-() const {
    Derived result;
    std::transform(data_, data_ + N, result.data_, std::negate<>());
    return result;
  }

  Derived operator*(const Derived& other) const {
    Derived result;
    std::transform(data_, data_ + N, other.data_, result.data_,
                   std::multiplies<>());
    return result;
  }

  Derived operator/(const Derived& other) const {
    Derived result;
    std::transform(data_, data_ + N, other.data_, result.data_,
                   std::divides<>());
    return result;
  }

  // 6. Arithmetic Operators (Vector-Scalar)
  Derived operator*(const T& scalar) const {
    Derived result;
    std::transform(data_, data_ + N, result.data_,
                   [scalar](const T& val) { return val * scalar; });
    return result;
  }

  Derived operator/(const T& scalar) const {
    Derived result;
    T inv_scalar = T{1} / scalar;
    std::transform(data_, data_ + N, result.data_,
                   [inv_scalar](const T& val) { return val * inv_scalar; });
    return result;
  }

  // 7. Compound Assignment Operators
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

  Derived& operator/=(const T& scalar) {
    T inv_scalar = T{1} / scalar;
    std::transform(data_, data_ + N, data_,
                   [inv_scalar](const T& val) { return val * inv_scalar; });
    return static_cast<Derived&>(*this);
  }

  // 8. Comparison Operators
  bool operator==(const Derived& other) const {
    return this == &other || std::equal(data_, data_ + N, other.data_);
  }

  // 9. Methods
  void fill(const T& value) { std::fill(data_, data_ + N, value); }

  T dot(const Derived& other) const {
    return std::inner_product(data_, data_ + N, other.data_, T{0});
  }

  T length_sqr() const { return dot(static_cast<const Derived&>(*this)); }
  T norm() const { return std::sqrt(length_sqr()); }

  void normalize() {
    T n = norm();
    if (std::abs(n) < epsilon()) {
      return;
    }
    T inv_n = T{1} / n;
    for (uint32_t i = 0; i < N; ++i) {
      data_[i] *= inv_n;
    }
  }

  Derived normalized() const {
    Derived result = static_cast<const Derived&>(*this);
    result.normalize();
    return result;
  }

  constexpr uint32_t size() const { return N; }

  // 10. Friend Functions
  friend std::ostream& operator<<(std::ostream& os, const Self& vec) {
    os << "[";
    for (uint32_t i = 0; i < N; ++i) {
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
// Vec
// ===========================
template <typename T, uint32_t N>
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

  Vec(T x, T y) : Base({x, y}) {};

  // Conversion constructors
  explicit Vec(const Vec<T, 3>& v) : Base({v.x(), v.y()}) {}
  explicit Vec(const Vec<T, 4>& v) : Base({v.x(), v.y()}) {}

  T& x() { return this->data_[0]; }
  const T& x() const { return this->data_[0]; }
  T& y() { return this->data_[1]; }
  const T& y() const { return this->data_[1]; }
};

// ===========================
// Specialization for Vec<T, 3>
// ===========================
template <typename T>
struct Vec<T, 3> : public VecBase<T, 3, Vec<T, 3>> {
  using Base = VecBase<T, 3, Vec<T, 3>>;
  using Base::Base;

  Vec(T x, T y, T z) : Base({x, y, z}) {};

  // Conversion constructors
  Vec(const Vec<T, 2>& v, T z) : Base({v.x(), v.y(), z}) {}
  explicit Vec(const Vec<T, 4>& v) : Base({v.x(), v.y(), v.z()}) {}

  T& x() { return this->data_[0]; }
  const T& x() const { return this->data_[0]; }
  T& y() { return this->data_[1]; }
  const T& y() const { return this->data_[1]; }
  T& z() { return this->data_[2]; }
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

  Vec(T x, T y, T z, T w) : Base({x, y, z, w}) {};

  // Conversion constructors
  Vec(const Vec<T, 3>& v, T w) : Base({v.x(), v.y(), v.z(), w}) {}
  explicit Vec(const Vec<T, 2>& v, T z, T w) : Base({v.x(), v.y(), z, w}) {}

  T& x() { return this->data_[0]; }
  const T& x() const { return this->data_[0]; }
  T& y() { return this->data_[1]; }
  const T& y() const { return this->data_[1]; }
  T& z() { return this->data_[2]; }
  const T& z() const { return this->data_[2]; }
  T& w() { return this->data_[3]; }
  const T& w() const { return this->data_[3]; }
};

// ===========================
// Free Functions
// ===========================
template <typename T, uint32_t N>
Vec<T, N> operator*(const T& scalar, const Vec<T, N>& vec) {
  return vec * scalar;
}

template <typename T, uint32_t N>
Vec<T, N> lerp(const Vec<T, N>& a, const Vec<T, N>& b, T t) {
  return a * (1 - t) + b * t;
}

// ===========================
// Type Aliases
// ===========================
using Vec2f = Vec<float, 2>;
using Vec3f = Vec<float, 3>;
using Vec4f = Vec<float, 4>;

using Vec2d = Vec<double, 2>;
using Vec3d = Vec<double, 3>;
using Vec4d = Vec<double, 4>;

using Vec2i = Vec<int, 2>;
using Vec3i = Vec<int, 3>;
using Vec4i = Vec<int, 4>;

using Vec2u = Vec<uint32_t, 2>;
using Vec3u = Vec<uint32_t, 3>;
using Vec4u = Vec<uint32_t, 4>;

}  // namespace math
}  // namespace soft_renderer