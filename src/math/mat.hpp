#pragma once
#include <cassert>
#include <cmath>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <type_traits>

#include "vec.hpp"

// 不同于VecBase<T, N, Derived>, 这里使用MatBase<Derived>
// MatBase并不存储数据，只定义行为，具体实现由派生类提供
// 有点像 Rust 中的 trait
// 之所以这么做，是用MatBase<T, M, N,
// Derived>定义的话不好拿返回类型，丧失了定义Derived的优势
// PS:
// 我目前这样写其实是没体现CRTP的优势的，因为我只写了*和transpose，而且没做什么偏特化，主要是学习目的
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

template <typename T, u32 M, u32 N>
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
  static constexpr u32 Rows = M;
  static constexpr u32 Cols = N;

  // 2. Constructors
  Mat() = default;
  Mat(std::initializer_list<ColVecType> list) {
    assert(list.size() == N);
    std::copy(list.begin(), list.end(), cols_);
  }

  // 3. Static Factory Methods
  static Self identity() {
    Self result;
    u32 min_dim = (M < N) ? M : N;
    for (u32 i = 0; i < min_dim; ++i) {
      result.cols_[i][i] = T{1};
    }
    return result;
  }

  // 4. Accessors
  ColVecType& operator[](u32 index) {
    assert(index < N);
    return cols_[index];
  }
  const ColVecType& operator[](u32 index) const {
    assert(index < N);
    return cols_[index];
  }

  T& operator()(u32 row, u32 col) {
    assert(row < M && col < N);
    return cols_[col][row];
  }
  const T& operator()(u32 row, u32 col) const {
    assert(row < M && col < N);
    return cols_[col][row];
  }

  RowVecType row(u32 index) {
    assert(index < M);
    RowVecType row;
    for (u32 i = 0; i < N; ++i) {
      row[i] = cols_[i][index];
    }
    return row;
  }
  const RowVecType row(u32 index) const {
    assert(index < M);
    RowVecType row;
    for (u32 i = 0; i < N; ++i) {
      row[i] = cols_[i][index];
    }
    return row;
  }

  // 5. Arithmetic Operators
  Self operator+(const Self& other) const {
    Self result;
    for (u32 i = 0; i < N; ++i) {
      result.cols_[i] = cols_[i] + other.cols_[i];
    }
    return result;
  }

  Self operator-(const Self& other) const {
    Self result;
    for (u32 i = 0; i < N; ++i) {
      result.cols_[i] = cols_[i] - other.cols_[i];
    }
    return result;
  }

  Self operator*(const T& scalar) const {
    Self result;
    for (u32 i = 0; i < N; ++i) {
      result.cols_[i] = cols_[i] * scalar;
    }
    return result;
  }

  // 6. Compound Assignment Operators
  Self& operator+=(const Self& other) {
    for (u32 i = 0; i < N; ++i) {
      cols_[i] += other.cols_[i];
    }
    return *this;
  }

  Self& operator-=(const Self& other) {
    for (u32 i = 0; i < N; ++i) {
      cols_[i] -= other.cols_[i];
    }
    return *this;
  }

  Self& operator*=(const T& scalar) {
    for (u32 i = 0; i < N; ++i) {
      cols_[i] *= scalar;
    }
    return *this;
  }

  // 7. Comparison Operators
  bool operator==(const Self& other) const {
    if constexpr (std::is_floating_point_v<T>) {
      for (u32 i = 0; i < N; ++i) {
        for (u32 j = 0; j < M; ++j) {
          if (std::abs(cols_[i][j] - other.cols_[i][j]) > epsilon()) {
            return false;
          }
        }
      }
      return true;
    } else {
      for (u32 i = 0; i < N; ++i) {
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
    for (u32 i = 0; i < N; ++i) cols_[i].fill(value);
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

  template <u32 BlockRows, u32 BlockCols>
  Mat<T, BlockRows, BlockCols> topLeftBlock() const {
    static_assert(M >= BlockRows && N >= BlockCols,
                  "Block size exceeds matrix dimensions");
    Mat<T, BlockRows, BlockCols> result;
    for (u32 i = 0; i < BlockRows; ++i) {
      for (u32 j = 0; j < BlockCols; ++j) {
        result(i, j) = cols_[j][i];
      }
    }
    return result;
  }

  template <u32 BlockRows, u32 BlockCols>
  Mat<T, BlockRows, BlockCols> topRightBlock() const {
    static_assert(M >= BlockRows && N >= BlockCols);
    Mat<T, BlockRows, BlockCols> result;
    for (u32 i = 0; i < BlockRows; ++i) {
      for (u32 j = 0; j < BlockCols; ++j) {
        result(i, j) = cols_[N - BlockCols + j][i];
      }
    }
    return result;
  }

  Mat<T, 3, 3> rotation() const {
    static_assert(M >= 3 && N >= 3, "Matrix too small to extract rotation");
    return topLeftBlock<3, 3>();
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

  Mat<T, M - 1, N - 1> minor(u32 row, u32 col) const {
    Mat<T, M - 1, N - 1> result;
    for (u32 i = 0; i < M; ++i) {
      for (u32 j = 0; j < N; ++j) {
        if (i == row || j == col) continue;
        u32 row_idx = i < row ? i : i - 1;
        u32 col_idx = j < col ? j : j - 1;
        result(row_idx, col_idx) = cols_[j][i];
      }
    }
    return result;
  }

  Scalar determinant() const {
    // 大规模应该用LU分解
    // 这里用伴随矩阵和行列式来计算，直接硬计算效率更高，但出于学习目的，递归一下
    static_assert(M == N, "Determinant is only defined for square matrices.");
    static_assert(
        M <= 4,
        "Determinant is only defined for 1x1, 2x2, 3x3, or 4x4 matrices.");
    if constexpr (M == 1) {
      return cols_[0][0];
    } else {
      Scalar result = T{0};
      for (u32 i = 0; i < N; ++i) {
        Scalar cofactor = (i % 2 == 0) ? 1 : -1;
        Mat<T, M - 1, M - 1> minor = this->minor(0, i);
        result += cofactor * cols_[i][0] * minor.determinant();
      }
      return result;
    }
  }

  Mat<T, M, M> inverse() const {
    // 同determinant，用行列式和伴随矩阵来算，复杂度n!
    static_assert(M == N, "Inverse is only defined for square matrices.");
    static_assert(
        M <= 4, "Inverse is only defined for 1x1, 2x2, 3x3, or 4x4 matrices.");
    if constexpr (M == 1) {
      return Mat<T, 1, 1>{1 / cols_[0][0]};
    }

    T det = this->determinant();
    if (std::abs(det) < epsilon()) {
      throw std::runtime_error("Matrix is singular and cannot be inverted.");
    }
    T inv_det = T{1} / det;
    Mat<T, M, M> result;
    for (u32 i = 0; i < M; ++i) {
      for (u32 j = 0; j < N; ++j) {
        Mat<T, M - 1, M - 1> cofactor_matrix = this->minor(i, j);
        Scalar cofactor = (i + j) % 2 == 0 ? 1 : -1;
        result(i, j) = cofactor * cofactor_matrix.determinant();
      }
    }
    return result.transpose() * inv_det;
  }

  // 9. Underlying CRTP implementations
  Mat<T, N, M> _transpose() const {
    Mat<T, N, M> result;
    for (u32 i = 0; i < M; ++i) {
      for (u32 j = 0; j < N; ++j) {
        result[i][j] = cols_[j][i];
      }
    }
    return result;
  }

  template <u32 K>
  Mat<T, M, K> _multiply(const Mat<T, N, K>& other) const {
    Mat<T, M, K> result;
    for (u32 j = 0; j < K; ++j) {
      result[j] = (*this) * other[j];
    }
    return result;
  }

  ColVecType _multiply(const Vec<T, N>& vec) const {
    ColVecType result{};
    for (u32 j = 0; j < N; ++j) {
      result += cols_[j] * vec[j];
    }
    return result;
  }

  // 10. Friend Functions
  friend std::ostream& operator<<(std::ostream& os, const Self& mat) {
    os << "[\n";
    for (u32 i = 0; i < M; ++i) {
      os << "  [";
      for (u32 j = 0; j < N; ++j) {
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

// ===========================
// Free Functions
// ===========================
template <typename T, u32 M, u32 N>
Mat<T, M, N> operator*(const T& scalar, const Mat<T, M, N>& mat) {
  return mat * scalar;
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

using Mat2i = Mat<int, 2, 2>;
using Mat3i = Mat<int, 3, 3>;
using Mat4i = Mat<int, 4, 4>;