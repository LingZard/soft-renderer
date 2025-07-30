#pragma once

#include <cmath>
#include <numeric>

#include "mat.hpp"
#include "vec.hpp"

namespace soft_renderer {
namespace math {

// Forward declaration
template <typename T>
class UnitQuat;

// ===========================
// Quat: General Quaternion Class
// ===========================
template <typename T>
class Quat {
 private:
  Vec<T, 4> vec_;

 public:
  // 1. Constructors
  Quat() : vec_{1, 0, 0, 0} {}
  Quat(const Vec<T, 4>& vec) : vec_(vec) {}
  Quat(T w, T x, T y, T z) : vec_{w, x, y, z} {}
  Quat(T scalar, const Vec<T, 3>& vec)
      : vec_{scalar, vec.x(), vec.y(), vec.z()} {}

  // 2. Accessors
  T& w() { return vec_[0]; }
  T& x() { return vec_[1]; }
  T& y() { return vec_[2]; }
  T& z() { return vec_[3]; }
  const T& w() const { return vec_[0]; }
  const T& x() const { return vec_[1]; }
  const T& y() const { return vec_[2]; }
  const T& z() const { return vec_[3]; }
  const Vec<T, 4>& as_vec() const { return vec_; }

  // 3. Arithmetic Operators (reusing Vec methods)
  Quat operator+(const Quat& other) const { return Quat(vec_ + other.vec_); }
  Quat operator-(const Quat& other) const { return Quat(vec_ - other.vec_); }
  Quat operator*(T scalar) const { return Quat(vec_ * scalar); }

  Quat operator*(const Quat& other) const {
    // Decompose into scalar and vector parts to show the logic
    T s1 = this->w();
    Vec<T, 3> v1 = {this->x(), this->y(), this->z()};
    T s2 = other.w();
    Vec<T, 3> v2 = {other.x(), other.y(), other.z()};

    // Hamilton product formula: q1 * q2 = (s1s2 - v1.v2, s1v2 + s2v1 + v1xv2)
    T new_s = s1 * s2 - v1.dot(v2);
    Vec<T, 3> new_v = v2 * s1 + v1 * s2 + v1.cross(v2);

    return Quat(new_s, new_v);
  }

  // 4. Compound Assignment Operators
  Quat& operator+=(const Quat& other) {
    vec_ += other.vec_;
    return *this;
  }
  Quat& operator-=(const Quat& other) {
    vec_ -= other.vec_;
    return *this;
  }
  Quat& operator*=(const Quat& other) {
    *this = *this * other;
    return *this;
  }
  Quat& operator*=(T scalar) {
    vec_ *= scalar;
    return *this;
  }

  // 5. Methods
  T norm_sqr() const { return vec_.length_sqr(); }
  T norm() const { return vec_.norm(); }
  Quat conjugate() const { return Quat(w(), -x(), -y(), -z()); }

  Quat inverse() const {
    T n_sqr = norm_sqr();
    if (n_sqr > std::numeric_limits<T>::epsilon()) {
      return conjugate() * (static_cast<T>(1) / n_sqr);
    }
    return Quat();  // Or handle error
  }

  UnitQuat<T> normalized() const;
};

// ===========================
// UnitQuat: Unit Quaternion Class (for rotations)
// ===========================
template <typename T>
class UnitQuat {
 private:
  Quat<T> q_;

 public:
  // 1. Constructors
  UnitQuat() : q_(1, 0, 0, 0) {}

  UnitQuat(const Vec<T, 3>& axis, T angle) {
    T half_angle = angle * T{0.5};
    T sin_half = std::sin(half_angle);
    T w = std::cos(half_angle);
    Vec<T, 3> v = axis.normalized() * sin_half;
    q_ = Quat<T>(w, v);
  }

  explicit UnitQuat(const Quat<T>& q) {
    q_ = q * (static_cast<T>(1) / q.norm());
  }

  // 2. Accessors
  const T& w() const { return q_.w(); }
  const T& x() const { return q_.x(); }
  const T& y() const { return q_.y(); }
  const T& z() const { return q_.z(); }

  // 3. Rotation-related Methods
  UnitQuat inverse() const { return UnitQuat(q_.conjugate()); }

  Vec<T, 3> rotate(const Vec<T, 3>& v) const {
    Quat<T> v_quat(0, v);
    Quat<T> result_quat = q_ * v_quat * q_.conjugate();
    return Vec<T, 3>{result_quat.x(), result_quat.y(), result_quat.z()};
  }

  Mat<T, 3, 3> to_rotation_matrix() const {
    Mat<T, 3, 3> result;
    result[0] = this->rotate(Vec<T, 3>{1, 0, 0});
    result[1] = this->rotate(Vec<T, 3>{0, 1, 0});
    result[2] = this->rotate(Vec<T, 3>{0, 0, 1});
    return result;
  }

  Mat<T, 4, 4> to_transform_matrix() const {
    Mat<T, 3, 3> rot_mat = this->to_rotation_matrix();
    Mat<T, 4, 4> result = Mat<T, 4, 4>::identity();
    for (uint32_t i = 0; i < 3; ++i) {
      for (uint32_t j = 0; j < 3; ++j) {
        result(i, j) = rot_mat(i, j);
      }
    }
    return result;
  }

  // 4. Interpolation
  static UnitQuat slerp(UnitQuat q1, UnitQuat q2, T t) {
    T cos_theta = q1.q_.w() * q2.q_.w() + q1.q_.x() * q2.q_.x() +
                  q1.q_.y() * q2.q_.y() + q1.q_.z() * q2.q_.z();
    // To ensure the shortest path, we flip one quaternion if the dot product is
    // negative. This brings the angle between them to be <= 90 degrees.
    if (cos_theta < 0) {
      q2.q_ = q2.q_ * static_cast<T>(-1);
      cos_theta = -cos_theta;
    }
    // If the angle is very small, `sin(theta)` will be close to zero, leading
    // to numerical instability from division by zero. In this case, we can
    // fall back to linear interpolation (nlerp), which is a good
    // approximation for small angles and is much faster and more stable.
    // The result just needs to be normalized.
    if (cos_theta > static_cast<T>(1) - std::numeric_limits<T>::epsilon()) {
      Quat<T> result_q = q1.q_ * (static_cast<T>(1) - t) + q2.q_ * t;
      return UnitQuat(result_q.normalized().q_);
    }

    T theta = std::acos(cos_theta);
    T sin_theta = std::sin(theta);

    T w1 = std::sin((static_cast<T>(1) - t) * theta) / sin_theta;
    T w2 = std::sin(t * theta) / sin_theta;

    Quat<T> result_q = q1.q_ * w1 + q2.q_ * w2;
    return UnitQuat(result_q);
  }

  // 5. Operators
  UnitQuat operator*(const UnitQuat& other) const {
    return UnitQuat(q_ * other.q_);
  }

  // 3. Static Factory Methods
  static UnitQuat identity() { return UnitQuat(); }

  static UnitQuat from_axis_angle(const Vec<T, 3>& axis, T angle_rad) {
    return UnitQuat(axis, angle_rad);
  }
};

// ===========================
// Quat Method Implementation (due to dependency on UnitQuat)
// ===========================
template <typename T>
UnitQuat<T> Quat<T>::normalized() const {
  return UnitQuat<T>(*this);
}

// ===========================
// Free Functions
// ===========================
template <typename T>
Quat<T> operator*(T scalar, const Quat<T>& q) {
  return q * scalar;
}

// ===========================
// Type Aliases
// ===========================
using Quatf = Quat<float>;
using Quatd = Quat<double>;
using UnitQuatf = UnitQuat<float>;
using UnitQuatd = UnitQuat<double>;

}  // namespace math
}  // namespace soft_renderer