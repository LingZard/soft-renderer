#pragma once
#include <cassert>
#include <cmath>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <type_traits>

#include "vec.hpp"

namespace soft_renderer {
namespace math {

// A bit different from VecBase, here we just use MatBase<Derived>.
// MatBase itself doesn't store data; it only defines behavior, with the
// actual implementation provided by the derived class (a bit like a Rust
// trait). The reason for this design is that using MatBase<T, M, N, Derived>
// would make it difficult to deduce return types, negating some benefits of
// CRTP.
//
// P.S. The current implementation doesn't fully leverage CRTP's power, as it
// only includes basic operators like `*` and `transpose` without partial
// specializations. This is primarily for learning purposes.
template <typename Derived>
struct MatBase {
 public:
  Derived& derived() { return static_cast<Derived&>(*this); }
  const Derived& derived() const { return static_cast<const Derived&>(*this); }

  template <typename OtherDerived>
  auto operator*(const OtherDerived& other) const {
    return derived()._multiply(other);
  }

  auto transpose() const { return derived()._transpose(); }
};

template <typename T, uint32_t M, uint32_t N>
struct Mat : public MatBase<Mat<T, M, N>> {
 private:
  using Self = Mat<T, M, N>;
  using ColVecType = Vec<T, M>;
  using RowVecType = Vec<T, N>;
  ColVecType cols_[N]{};

  static constexpr T epsilon() {
    if constexpr (std::is_floating_point_v<T>) {
      return std::numeric_limits<T>::epsilon() * 10;
    } else {
      return T{0};
    }
  }

 public:
  // 1. Types and Constants
  using MatBase<Self>::operator*;
  using Scalar = T;
  static constexpr uint32_t Rows = M;
  static constexpr uint32_t Cols = N;

  // 2. Constructors
  Mat() = default;
  Mat(std::initializer_list<ColVecType> list) {
    assert(list.size() == N);
    std::copy(list.begin(), list.end(), cols_);
  }

  // 3. Static Factory Methods
  static Self identity() {
    Self result;
    uint32_t min_dim = (M < N) ? M : N;
    for (uint32_t i = 0; i < min_dim; ++i) {
      result.cols_[i][i] = T{1};
    }
    return result;
  }

  // 4. Accessors
  ColVecType& operator[](uint32_t index) {
    assert(index < N);
    return cols_[index];
  }
  const ColVecType& operator[](uint32_t index) const {
    assert(index < N);
    return cols_[index];
  }

  T& operator()(uint32_t row, uint32_t col) {
    assert(row < M && col < N);
    return cols_[col][row];
  }
  const T& operator()(uint32_t row, uint32_t col) const {
    assert(row < M && col < N);
    return cols_[col][row];
  }

  RowVecType row(uint32_t index) {
    assert(index < M);
    RowVecType row;
    for (uint32_t i = 0; i < N; ++i) {
      row[i] = cols_[i][index];
    }
    return row;
  }
  const RowVecType row(uint32_t index) const {
    assert(index < M);
    RowVecType row;
    for (uint32_t i = 0; i < N; ++i) {
      row[i] = cols_[i][index];
    }
    return row;
  }
  ColVecType col(uint32_t index) {
    assert(index < N);
    return cols_[index];
  }
  const ColVecType col(uint32_t index) const {
    assert(index < N);
    return cols_[index];
  }

  // 5. Public Methods
  void set_col(uint32_t index, const ColVecType& col_vec) {
    assert(index < N);
    cols_[index] = col_vec;
  }

  // 6. Arithmetic Operators
  Self operator+(const Self& other) const {
    Self result;
    for (uint32_t i = 0; i < N; ++i) {
      result.cols_[i] = cols_[i] + other.cols_[i];
    }
    return result;
  }

  Self operator-(const Self& other) const {
    Self result;
    for (uint32_t i = 0; i < N; ++i) {
      result.cols_[i] = cols_[i] - other.cols_[i];
    }
    return result;
  }

  Self operator*(const T& scalar) const {
    Self result;
    for (uint32_t i = 0; i < N; ++i) {
      result.cols_[i] = cols_[i] * scalar;
    }
    return result;
  }

  // 7. Compound Assignment Operators
  Self& operator+=(const Self& other) {
    for (uint32_t i = 0; i < N; ++i) {
      cols_[i] += other.cols_[i];
    }
    return *this;
  }

  Self& operator-=(const Self& other) {
    for (uint32_t i = 0; i < N; ++i) {
      cols_[i] -= other.cols_[i];
    }
    return *this;
  }

  Self& operator*=(const T& scalar) {
    for (uint32_t i = 0; i < N; ++i) {
      cols_[i] *= scalar;
    }
    return *this;
  }

  // 8. Comparison Operators
  bool operator==(const Self& other) const {
    if constexpr (std::is_floating_point_v<T>) {
      for (uint32_t i = 0; i < N; ++i) {
        for (uint32_t j = 0; j < M; ++j) {
          if (std::abs(cols_[i][j] - other.cols_[i][j]) > epsilon()) {
            return false;
          }
        }
      }
      return true;
    } else {
      for (uint32_t i = 0; i < N; ++i) {
        if (!(cols_[i] == other.cols_[i])) {
          return false;
        }
      }
      return true;
    }
  }

  bool operator!=(const Self& other) const { return !(*this == other); }

  // 8. Methods
  void fill(const T& value) {
    for (uint32_t i = 0; i < N; ++i) cols_[i].fill(value);
  }

  // 8.1. Graphics-specific Methods
  Vec<T, 3> scale() const {
    static_assert(M >= 3 && N >= 3, "Matrix too small to extract scale");
    return Vec<T, 3>{Vec<T, 3>{cols_[0][0], cols_[0][1], cols_[0][2]}.norm(),
                     Vec<T, 3>{cols_[1][0], cols_[1][1], cols_[1][2]}.norm(),
                     Vec<T, 3>{cols_[2][0], cols_[2][1], cols_[2][2]}.norm()};
  }

  void set_translation(const Vec<T, 3>& trans) {
    static_assert(M >= 3 && N >= 4, "Matrix too small for translation");
    cols_[3][0] = trans.x();
    cols_[3][1] = trans.y();
    cols_[3][2] = trans.z();
  }

  template <uint32_t BlockRows, uint32_t BlockCols>
  Mat<T, BlockRows, BlockCols> block() const {
    static_assert(M >= BlockRows && N >= BlockCols,
                  "Block size exceeds matrix dimensions");
    Mat<T, BlockRows, BlockCols> result;
    for (uint32_t i = 0; i < BlockRows; ++i) {
      for (uint32_t j = 0; j < BlockCols; ++j) {
        result(i, j) = cols_[j][i];
      }
    }
    return result;
  }

  Mat<T, 3, 3> rotation() const {
    static_assert(M >= 3 && N >= 3, "Matrix too small to extract rotation");
    return block<3, 3>();
  }

  Vec<T, 3> translation() const {
    static_assert(M >= 3 && N >= 4, "Matrix too small to extract translation");
    return Vec<T, 3>{cols_[3][0], cols_[3][1], cols_[3][2]};
  }

  // 8.2. Linear Algebra Methods
  bool is_invertible() const {
    static_assert(M == N, "Invertibility check only for square matrices");
    return std::abs(determinant()) >= epsilon();
  }

  Mat<T, M - 1, N - 1> minor(uint32_t row, uint32_t col) const {
    Mat<T, M - 1, N - 1> result;
    uint32_t res_i = 0;
    for (uint32_t i = 0; i < M; ++i) {
      if (i == row) continue;
      uint32_t res_j = 0;
      for (uint32_t j = 0; j < N; ++j) {
        if (j == col) continue;
        result(res_i, res_j) = cols_[j][i];
        res_j++;
      }
      res_i++;
    }
    return result;
  }

  // For large matrices, LU decomposition should be used.
  // Here we use recursive expansion by minors for learning purposes,
  // even though a direct, hard-coded calculation would be more efficient for
  // small matrices.
  Scalar determinant() const {
    static_assert(M == N, "Determinant is only defined for square matrices.");
    static_assert(M <= 4,
                  "Recursive determinant is only supported for up to 4x4.");
    if constexpr (M == 1) {
      return cols_[0][0];
    } else {
      Scalar result = T{0};
      for (uint32_t i = 0; i < N; ++i) {
        Scalar cofactor_sign = (i % 2 == 0) ? 1 : -1;
        result += cofactor_sign * cols_[i][0] * this->minor(0, i).determinant();
      }
      return result;
    }
  }

  // Same as determinant, this calculates the inverse using the adjugate
  // matrix method, which has O(n!) complexity. This is purely for learning
  // purposes.
  Mat<T, M, M> inverse() const {
    static_assert(M == N, "Inverse is only defined for square matrices.");
    static_assert(M <= 4, "Recursive inverse is only supported for up to 4x4.");

    T det = this->determinant();
    if (std::abs(det) < epsilon()) {
      throw std::runtime_error("Matrix is singular and cannot be inverted.");
    }

    if constexpr (M == 1) {
      return Mat<T, 1, 1>{{T{1} / det}};
    }

    T inv_det = T{1} / det;
    Mat<T, M, M> adjugate;
    for (uint32_t i = 0; i < M; ++i) {
      for (uint32_t j = 0; j < N; ++j) {
        Scalar cofactor_sign = ((i + j) % 2 == 0) ? 1 : -1;
        adjugate(j, i) = cofactor_sign * this->minor(i, j).determinant();
      }
    }
    return adjugate * inv_det;
  }

  // 9. Underlying CRTP implementations
  Mat<T, N, M> _transpose() const {
    Mat<T, N, M> result;
    for (uint32_t i = 0; i < M; ++i) {
      for (uint32_t j = 0; j < N; ++j) {
        result[i][j] = cols_[j][i];
      }
    }
    return result;
  }

  template <uint32_t K>
  Mat<T, M, K> _multiply(const Mat<T, N, K>& other) const {
    Mat<T, M, K> result;
    for (uint32_t j = 0; j < K; ++j) {
      result[j] = (*this) * other[j];
    }
    return result;
  }

  ColVecType _multiply(const Vec<T, N>& vec) const {
    ColVecType result{};
    for (uint32_t j = 0; j < N; ++j) {
      result += cols_[j] * vec[j];
    }
    return result;
  }

  // 10. Friend Functions
  friend std::ostream& operator<<(std::ostream& os, const Self& mat) {
    os << "[\n";
    for (uint32_t i = 0; i < M; ++i) {
      os << "  [";
      for (uint32_t j = 0; j < N; ++j) {
        os << mat(i, j);
        if (j < N - 1) {
          os << ", ";
        }
      }
      os << "]";
      if (i < M - 1) {
        os << ",\n";
      }
    }
    os << "\n]";
    return os;
  }
};

template <typename T, uint32_t M, uint32_t N>
Mat<T, M, N> operator*(const T& scalar, const Mat<T, M, N>& mat) {
  return mat * scalar;
}

// ===========================
// Graphics Transformation Functions
// ===========================

/// @brief Creates a 4x4 translation matrix.
/// @param trans The translation vector.
/// @return A 4x4 translation matrix.
template <typename T>
Mat<T, 4, 4> create_translation(const Vec<T, 3>& trans) {
  Mat<T, 4, 4> result = Mat<T, 4, 4>::identity();
  result(0, 3) = trans.x();
  result(1, 3) = trans.y();
  result(2, 3) = trans.z();
  return result;
}

/// @brief Creates a 4x4 scaling matrix.
/// @param scale The scaling vector.
/// @return A 4x4 scaling matrix.
template <typename T>
Mat<T, 4, 4> create_scale(const Vec<T, 3>& scale) {
  Mat<T, 4, 4> result = Mat<T, 4, 4>::identity();
  result(0, 0) = scale.x();
  result(1, 1) = scale.y();
  result(2, 2) = scale.z();
  return result;
}

/// @brief Creates a 4x4 rotation matrix from an axis and an angle.
/// @param axis The rotation axis (should be normalized).
/// @param angle_rad The rotation angle in radians.
/// @return A 4x4 rotation matrix.
template <typename T>
Mat<T, 4, 4> create_rotation(const Vec<T, 3>& axis, T angle_rad) {
  T const c = std::cos(angle_rad);
  T const s = std::sin(angle_rad);
  Vec<T, 3> k = axis.normalized();

  auto rotate_vec = [&](const Vec<T, 3>& v) {
    // Rodrigues' rotation formula:
    // v_rot = v*cos(t) + (k x v)*sin(t) + k*(k.v)*(1-cos(t))
    return v * c + k.cross(v) * s + k * k.dot(v) * (T{1} - c);
  };

  Vec<T, 3> new_i = rotate_vec({1, 0, 0});
  Vec<T, 3> new_j = rotate_vec({0, 1, 0});
  Vec<T, 3> new_k = rotate_vec({0, 0, 1});

  Mat<T, 4, 4> result = Mat<T, 4, 4>::identity();
  result[0] = Vec<T, 4>(new_i, 0);
  result[1] = Vec<T, 4>(new_j, 0);
  result[2] = Vec<T, 4>(new_k, 0);

  return result;
}

// We've implemented `look_at` here, but our Camera class uses quaternions
// directly for orientation, so this function won't actually be called in the
// current setup. It's here for completeness and testing purposes.
template <typename T>
Mat<T, 4, 4> look_at(const Vec<T, 3>& eye, const Vec<T, 3>& center,
                     const Vec<T, 3>& world_up) {
  // Using the standard OpenGL convention.
  // Some checks are omitted for brevity (e.g., eye != center, forward and up
  // are not parallel). The basis vectors are defined as right (local X), up
  // (local Y), and -forward (local Z).
  Vec<T, 3> forward = (center - eye).normalized();
  Vec<T, 3> right = forward.cross(world_up).normalized();
  Vec<T, 3> up = right.cross(forward).normalized();

  Mat<T, 4, 4> rotation;
  rotation.set_col(0, Vec<T, 4>(right, 0.0));
  rotation.set_col(1, Vec<T, 4>(up, 0.0));
  rotation.set_col(2, Vec<T, 4>(-forward, 0.0));
  rotation(3, 3) = 1.0;

  Mat<T, 4, 4> translation = create_translation(-eye);

  return rotation.transpose() * translation;
}

template <typename T>
Mat<T, 4, 4> perspective(T aspect_ratio, T fov_y_rad, T near, T far) {
  // Note: `near` and `far` are positive distances from the camera.
  Mat<T, 4, 4> result = Mat<T, 4, 4>::identity();
  T const f = 1.0 / std::tan(fov_y_rad / 2.0);
  T const a_inv = 1.0 / aspect_ratio;
  result(0, 0) = f * a_inv;
  result(1, 1) = f;
  result(2, 2) = (near + far) / (near - far);
  result(3, 2) = -1.0;
  result(2, 3) = 2.0 * near * far / (near - far);
  result(3, 3) = 0.0;
  return result;
}

template <typename T>
Mat<T, 4, 4> orthographic(T left, T right, T bottom, T top, T near, T far) {
  // Note: `left`, `right`, `bottom`, `top` are signed values, while `near` and
  // `far` are positive distances.
  Mat<T, 4, 4> result = Mat<T, 4, 4>::identity();
  result(0, 0) = 2.0 / (right - left);
  result(1, 1) = 2.0 / (top - bottom);
  result(2, 2) = -2.0 / (far - near);
  result(0, 3) = -(right + left) / (right - left);
  result(1, 3) = -(top + bottom) / (top - bottom);
  result(2, 3) = -(far + near) / (far - near);
  return result;
}

// ===========================
// Type Aliases
// ===========================
using Mat2f = Mat<float, 2, 2>;
using Mat3f = Mat<float, 3, 3>;
using Mat4f = Mat<float, 4, 4>;

using Mat2d = Mat<double, 2, 2>;
using Mat3d = Mat<double, 3, 3>;
using Mat4d = Mat<double, 4, 4>;

}  // namespace math
}  // namespace soft_renderer