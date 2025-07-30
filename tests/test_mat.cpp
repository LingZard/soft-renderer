#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include <type_traits>

#include "../src/core/math/mat.hpp"

using namespace soft_renderer::math;

TEST_CASE("Mat basic construction and access", "[mat-basic]") {
  // Default constructor
  Mat<float, 2, 2> m1;
  REQUIRE(m1(0, 0) == 0.0f);
  REQUIRE(m1(0, 1) == 0.0f);
  REQUIRE(m1(1, 0) == 0.0f);
  REQUIRE(m1(1, 1) == 0.0f);

  // Constructor with column vectors
  Vec<float, 2> col1{1.0f, 2.0f};
  Vec<float, 2> col2{3.0f, 4.0f};
  Mat<float, 2, 2> m2{col1, col2};

  REQUIRE(m2(0, 0) == 1.0f);
  REQUIRE(m2(1, 0) == 2.0f);
  REQUIRE(m2(0, 1) == 3.0f);
  REQUIRE(m2(1, 1) == 4.0f);

  // Column access
  REQUIRE(m2[0][0] == 1.0f);
  REQUIRE(m2[0][1] == 2.0f);
  REQUIRE(m2[1][0] == 3.0f);
  REQUIRE(m2[1][1] == 4.0f);

  // Row access
  auto row0 = m2.row(0);
  auto row1 = m2.row(1);
  REQUIRE(row0[0] == 1.0f);
  REQUIRE(row0[1] == 3.0f);
  REQUIRE(row1[0] == 2.0f);
  REQUIRE(row1[1] == 4.0f);
}

TEST_CASE("Mat identity matrix", "[mat-identity]") {
  auto id2 = Mat<float, 2, 2>::identity();
  REQUIRE(id2(0, 0) == 1.0f);
  REQUIRE(id2(0, 1) == 0.0f);
  REQUIRE(id2(1, 0) == 0.0f);
  REQUIRE(id2(1, 1) == 1.0f);

  auto id3 = Mat<float, 3, 3>::identity();
  REQUIRE(id3(0, 0) == 1.0f);
  REQUIRE(id3(1, 1) == 1.0f);
  REQUIRE(id3(2, 2) == 1.0f);
  REQUIRE(id3(0, 1) == 0.0f);
  REQUIRE(id3(0, 2) == 0.0f);
  REQUIRE(id3(1, 0) == 0.0f);

  auto id4 = Mat<float, 4, 4>::identity();
  REQUIRE(id4(0, 0) == 1.0f);
  REQUIRE(id4(1, 1) == 1.0f);
  REQUIRE(id4(2, 2) == 1.0f);
  REQUIRE(id4(3, 3) == 1.0f);
}

TEST_CASE("Mat arithmetic operations", "[mat-arithmetic]") {
  Vec<float, 2> col1{1.0f, 2.0f};
  Vec<float, 2> col2{3.0f, 4.0f};
  Mat<float, 2, 2> m1{col1, col2};

  Vec<float, 2> col3{5.0f, 6.0f};
  Vec<float, 2> col4{7.0f, 8.0f};
  Mat<float, 2, 2> m2{col3, col4};

  // Matrix addition
  auto sum = m1 + m2;
  REQUIRE(sum(0, 0) == 6.0f);
  REQUIRE(sum(1, 0) == 8.0f);
  REQUIRE(sum(0, 1) == 10.0f);
  REQUIRE(sum(1, 1) == 12.0f);

  // Matrix subtraction
  auto diff = m2 - m1;
  REQUIRE(diff(0, 0) == 4.0f);
  REQUIRE(diff(1, 0) == 4.0f);
  REQUIRE(diff(0, 1) == 4.0f);
  REQUIRE(diff(1, 1) == 4.0f);

  // Scalar multiplication
  auto scaled = m1 * 2.0f;
  REQUIRE(scaled(0, 0) == 2.0f);
  REQUIRE(scaled(1, 0) == 4.0f);
  REQUIRE(scaled(0, 1) == 6.0f);
  REQUIRE(scaled(1, 1) == 8.0f);

  // Left scalar multiplication
  auto scaled2 = 3.0f * m1;
  REQUIRE(scaled2(0, 0) == 3.0f);
  REQUIRE(scaled2(1, 0) == 6.0f);
  REQUIRE(scaled2(0, 1) == 9.0f);
  REQUIRE(scaled2(1, 1) == 12.0f);
}

TEST_CASE("Mat compound assignment operations", "[mat-compound]") {
  Vec<float, 2> col1{1.0f, 2.0f};
  Vec<float, 2> col2{3.0f, 4.0f};
  Mat<float, 2, 2> m1{col1, col2};

  Vec<float, 2> col3{5.0f, 6.0f};
  Vec<float, 2> col4{7.0f, 8.0f};
  Mat<float, 2, 2> m2{col3, col4};

  // +=
  m1 += m2;
  REQUIRE(m1(0, 0) == 6.0f);
  REQUIRE(m1(1, 0) == 8.0f);
  REQUIRE(m1(0, 1) == 10.0f);
  REQUIRE(m1(1, 1) == 12.0f);

  // -=
  m1 -= m2;
  REQUIRE(m1(0, 0) == 1.0f);
  REQUIRE(m1(1, 0) == 2.0f);
  REQUIRE(m1(0, 1) == 3.0f);
  REQUIRE(m1(1, 1) == 4.0f);

  // *=
  m1 *= 2.0f;
  REQUIRE(m1(0, 0) == 2.0f);
  REQUIRE(m1(1, 0) == 4.0f);
  REQUIRE(m1(0, 1) == 6.0f);
  REQUIRE(m1(1, 1) == 8.0f);
}

TEST_CASE("Mat transpose", "[mat-transpose]") {
  Vec<float, 2> col1{1.0f, 2.0f};
  Vec<float, 2> col2{3.0f, 4.0f};
  Mat<float, 2, 2> m{col1, col2};

  auto t = m.transpose();
  REQUIRE(t(0, 0) == 1.0f);
  REQUIRE(t(0, 1) == 2.0f);
  REQUIRE(t(1, 0) == 3.0f);
  REQUIRE(t(1, 1) == 4.0f);

  // Transpose of a non-square matrix
  Vec<float, 2> c1{1.0f, 2.0f};
  Vec<float, 2> c2{3.0f, 4.0f};
  Vec<float, 2> c3{5.0f, 6.0f};
  Mat<float, 2, 3> rect{c1, c2, c3};

  auto rect_t = rect.transpose();
  static_assert(std::is_same_v<decltype(rect_t), Mat<float, 3, 2>>);
  REQUIRE(rect_t(0, 0) == 1.0f);
  REQUIRE(rect_t(0, 1) == 2.0f);
  REQUIRE(rect_t(1, 0) == 3.0f);
  REQUIRE(rect_t(1, 1) == 4.0f);
  REQUIRE(rect_t(2, 0) == 5.0f);
  REQUIRE(rect_t(2, 1) == 6.0f);
}

TEST_CASE("Mat matrix multiplication", "[mat-multiply]") {
  // 2x2 matrix multiplication
  Vec<float, 2> col1{1.0f, 2.0f};
  Vec<float, 2> col2{3.0f, 4.0f};
  Mat<float, 2, 2> m1{col1, col2};

  Vec<float, 2> col3{5.0f, 6.0f};
  Vec<float, 2> col4{7.0f, 8.0f};
  Mat<float, 2, 2> m2{col3, col4};

  auto result = m1 * m2;
  // [1 3] * [5 7] = [1*5+3*6  1*7+3*8] = [23 31]
  // [2 4]   [6 8]   [2*5+4*6  2*7+4*8] = [34 46]
  REQUIRE(result(0, 0) == Catch::Approx(23.0f));
  REQUIRE(result(0, 1) == Catch::Approx(31.0f));
  REQUIRE(result(1, 0) == Catch::Approx(34.0f));
  REQUIRE(result(1, 1) == Catch::Approx(46.0f));

  // Multiplication by identity matrix
  auto id = Mat<float, 2, 2>::identity();
  auto id_result = m1 * id;
  REQUIRE(id_result(0, 0) == Catch::Approx(m1(0, 0)));
  REQUIRE(id_result(0, 1) == Catch::Approx(m1(0, 1)));
  REQUIRE(id_result(1, 0) == Catch::Approx(m1(1, 0)));
  REQUIRE(id_result(1, 1) == Catch::Approx(m1(1, 1)));
}

TEST_CASE("Matrix-vector multiplication", "[mat-vec-mul]") {
  Vec<float, 2> col1{1.0f, 2.0f};
  Vec<float, 2> col2{3.0f, 4.0f};
  Mat<float, 2, 2> m{col1, col2};

  Vec<float, 2> v{5.0f, 6.0f};
  auto result = m * v;

  // [1 3] * [5] = [1*5 + 3*6] = [23]
  // [2 4]   [6]   [2*5 + 4*6] = [34]
  REQUIRE(result[0] == Catch::Approx(23.0f));
  REQUIRE(result[1] == Catch::Approx(34.0f));
}

TEST_CASE("Mat determinant 2x2", "[mat-det-2x2]") {
  Vec<float, 2> c1{3.0f, 1.0f};
  Vec<float, 2> c2{2.0f, 4.0f};
  Mat<float, 2, 2> m{c1, c2};

  // det = 3*4 - 2*1 = 12 - 2 = 10
  REQUIRE(m.determinant() == Catch::Approx(10.0f));

  auto id = Mat<float, 2, 2>::identity();
  REQUIRE(id.determinant() == Catch::Approx(1.0f));
}

TEST_CASE("Mat determinant 3x3", "[mat-det-3x3]") {
  Vec<float, 3> col1{1.0f, 0.0f, 2.0f};
  Vec<float, 3> col2{-1.0f, 3.0f, 1.0f};
  Vec<float, 3> col3{2.0f, 1.0f, 0.0f};
  Mat<float, 3, 3> m{col1, col2, col3};

  // Manual calculation: det = 1*(3*0-1*1) - (-1)*(0*0-1*2) + 2*(0*1-3*2) = -1 -
  // 2 - 12 = -15
  REQUIRE(m.determinant() == Catch::Approx(-15.0f));
}

TEST_CASE("Mat inverse 2x2", "[mat-inverse-2x2]") {
  Vec<float, 2> col1{1.0f, 2.0f};
  Vec<float, 2> col2{3.0f, 4.0f};
  Mat<float, 2, 2> m{col1, col2};

  auto inv = m.inverse();
  auto result = m * inv;
  auto id = Mat<float, 2, 2>::identity();

  // Check that m * inv is approximately the identity matrix
  REQUIRE(result(0, 0) == Catch::Approx(id(0, 0)).margin(1e-6));
  REQUIRE(result(0, 1) == Catch::Approx(id(0, 1)).margin(1e-6));
  REQUIRE(result(1, 0) == Catch::Approx(id(1, 0)).margin(1e-6));
  REQUIRE(result(1, 1) == Catch::Approx(id(1, 1)).margin(1e-6));
}

TEST_CASE("Mat inverse 3x3", "[mat-inverse-3x3]") {
  Vec<float, 3> col1{1.f, 2.f, 3.f};
  Vec<float, 3> col2{0.f, 1.f, 4.f};
  Vec<float, 3> col3{5.f, 6.f, 0.f};
  Mat<float, 3, 3> m{col1, col2, col3};

  auto inv = m.inverse();
  auto result = m * inv;
  auto id = Mat<float, 3, 3>::identity();

  REQUIRE(result(0, 0) == Catch::Approx(id(0, 0)).margin(1e-6));
  REQUIRE(result(0, 1) == Catch::Approx(id(0, 1)).margin(1e-6));
  REQUIRE(result(0, 2) == Catch::Approx(id(0, 2)).margin(1e-6));
  REQUIRE(result(1, 0) == Catch::Approx(id(1, 0)).margin(1e-6));
  REQUIRE(result(1, 1) == Catch::Approx(id(1, 1)).margin(1e-6));
  REQUIRE(result(1, 2) == Catch::Approx(id(1, 2)).margin(1e-6));
  REQUIRE(result(2, 0) == Catch::Approx(id(2, 0)).margin(1e-6));
  REQUIRE(result(2, 1) == Catch::Approx(id(2, 1)).margin(1e-6));
  REQUIRE(result(2, 2) == Catch::Approx(id(2, 2)).margin(1e-6));
}

TEST_CASE("Mat inverse 4x4", "[mat-inverse-4x4]") {
  Vec<float, 4> col1{1.f, 0.f, 0.f, 0.f};
  Vec<float, 4> col2{0.f, 1.f, 0.f, 0.f};
  Vec<float, 4> col3{0.f, 0.f, 1.f, 0.f};
  Vec<float, 4> col4{1.f, 2.f, 3.f, 1.f};  // A simple translation matrix
  Mat<float, 4, 4> m{col1, col2, col3, col4};

  auto inv = m.inverse();
  auto result = m * inv;
  auto id = Mat<float, 4, 4>::identity();

  REQUIRE(result(0, 0) == Catch::Approx(id(0, 0)).margin(1e-6));
  REQUIRE(result(0, 1) == Catch::Approx(id(0, 1)).margin(1e-6));
  REQUIRE(result(0, 2) == Catch::Approx(id(0, 2)).margin(1e-6));
  REQUIRE(result(0, 3) == Catch::Approx(id(0, 3)).margin(1e-6));
  REQUIRE(result(1, 0) == Catch::Approx(id(1, 0)).margin(1e-6));
  REQUIRE(result(1, 1) == Catch::Approx(id(1, 1)).margin(1e-6));
  REQUIRE(result(1, 2) == Catch::Approx(id(1, 2)).margin(1e-6));
  REQUIRE(result(1, 3) == Catch::Approx(id(1, 3)).margin(1e-6));
  REQUIRE(result(2, 0) == Catch::Approx(id(2, 0)).margin(1e-6));
  REQUIRE(result(2, 1) == Catch::Approx(id(2, 1)).margin(1e-6));
  REQUIRE(result(2, 2) == Catch::Approx(id(2, 2)).margin(1e-6));
  REQUIRE(result(2, 3) == Catch::Approx(id(2, 3)).margin(1e-6));
  REQUIRE(result(3, 0) == Catch::Approx(id(3, 0)).margin(1e-6));
  REQUIRE(result(3, 1) == Catch::Approx(id(3, 1)).margin(1e-6));
  REQUIRE(result(3, 2) == Catch::Approx(id(3, 2)).margin(1e-6));
  REQUIRE(result(3, 3) == Catch::Approx(id(3, 3)).margin(1e-6));
}

TEST_CASE("Mat invertible check", "[mat-invertible]") {
  Vec<float, 2> col1{1.0f, 2.0f};
  Vec<float, 2> col2{2.0f, 4.0f};  // Linearly dependent columns
  Mat<float, 2, 2> singular{col1, col2};
  REQUIRE_FALSE(singular.is_invertible());

  Vec<float, 2> col3{1.0f, 2.0f};
  Vec<float, 2> col4{3.0f, 4.0f};
  Mat<float, 2, 2> non_singular{col3, col4};
  REQUIRE(non_singular.is_invertible());
}

TEST_CASE("Mat minor", "[mat-minor]") {
  Vec<float, 3> c1{1, 2, 3};
  Vec<float, 3> c2{4, 5, 6};
  Vec<float, 3> c3{7, 8, 9};
  Mat<float, 3, 3> m{c1, c2, c3};

  auto minor_00 = m.minor(0, 0);
  REQUIRE(minor_00(0, 0) == 5.0f);
  REQUIRE(minor_00(0, 1) == 8.0f);
  REQUIRE(minor_00(1, 0) == 6.0f);
  REQUIRE(minor_00(1, 1) == 9.0f);

  auto minor_11 = m.minor(1, 1);
  REQUIRE(minor_11(0, 0) == 1.0f);
  REQUIRE(minor_11(0, 1) == 7.0f);
  REQUIRE(minor_11(1, 0) == 3.0f);
  REQUIRE(minor_11(1, 1) == 9.0f);
}

TEST_CASE("Graphics transformation matrices", "[mat-graphics]") {
  SECTION("perspective matrix correctness") {
    double aspect = 16.0 / 9.0;
    double fov_y_rad = M_PI / 2.0;  // 90 degrees
    double near = 0.1;
    double far = 1000.0;
    Mat4d proj = perspective(aspect, fov_y_rad, near, far);

    // A point on the near plane should be transformed to NDC z = -1
    Vec4d point_on_near_view = {1.0, 1.0, -near, 1.0};
    Vec4d point_on_near_clip = proj * point_on_near_view;
    double z_ndc_near = point_on_near_clip.z() / point_on_near_clip.w();
    REQUIRE(z_ndc_near == Catch::Approx(-1.0));

    // A point on the far plane should be transformed to NDC z = +1
    Vec4d point_on_far_view = {10.0, 10.0, -far, 1.0};
    Vec4d point_on_far_clip = proj * point_on_far_view;
    double z_ndc_far = point_on_far_clip.z() / point_on_far_clip.w();
    REQUIRE(z_ndc_far == Catch::Approx(1.0));
  }

  SECTION("orthographic matrix correctness") {
    double left = -10, right = 10, bottom = -5, top = 5;
    double near = 0.1, far = 100.0;
    Mat4d proj = orthographic(left, right, bottom, top, near, far);

    // Point at bottom-left corner on near plane should map to NDC (-1, -1, -1)
    Vec4d point_bl_view = {left, bottom, -near, 1.0};
    Vec4d point_bl_ndc = proj * point_bl_view;
    REQUIRE(point_bl_ndc.x() == Catch::Approx(-1.0));
    REQUIRE(point_bl_ndc.y() == Catch::Approx(-1.0));
    REQUIRE(point_bl_ndc.z() == Catch::Approx(-1.0));
    REQUIRE(point_bl_ndc.w() == Catch::Approx(1.0));

    // Point at top-right corner on far plane should map to NDC (1, 1, 1)
    Vec4d point_tr_view = {right, top, -far, 1.0};
    Vec4d point_tr_ndc = proj * point_tr_view;
    REQUIRE(point_tr_ndc.x() == Catch::Approx(1.0));
    REQUIRE(point_tr_ndc.y() == Catch::Approx(1.0));
    REQUIRE(point_tr_ndc.z() == Catch::Approx(1.0));
    REQUIRE(point_tr_ndc.w() == Catch::Approx(1.0));
  }
}
