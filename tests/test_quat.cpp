#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include <cmath>
#include <type_traits>

#include "../src/core/math/quat.hpp"

using namespace soft_renderer::math;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

TEST_CASE("Quat basic construction and access", "[quat-basic]") {
  // Default constructor
  Quatf q1;
  REQUIRE(q1.w() == 1.0f);
  REQUIRE(q1.x() == 0.0f);
  REQUIRE(q1.y() == 0.0f);
  REQUIRE(q1.z() == 0.0f);

  // w, x, y, z constructor
  Quatf q2(1.0f, 2.0f, 3.0f, 4.0f);
  REQUIRE(q2.w() == 1.0f);
  REQUIRE(q2.x() == 2.0f);
  REQUIRE(q2.y() == 3.0f);
  REQUIRE(q2.z() == 4.0f);

  // scalar, vec3 constructor
  Vec3f v(2.0f, 3.0f, 4.0f);
  Quatf q3(1.0f, v);
  REQUIRE(q3.w() == 1.0f);
  REQUIRE(q3.x() == 2.0f);
  REQUIRE(q3.y() == 3.0f);
  REQUIRE(q3.z() == 4.0f);

  // Vec4 constructor
  Vec4f v4(1.0f, 2.0f, 3.0f, 4.0f);
  Quatf q4(v4);
  REQUIRE(q4.w() == 1.0f);
  REQUIRE(q4.x() == 2.0f);
  REQUIRE(q4.y() == 3.0f);
  REQUIRE(q4.z() == 4.0f);

  // as_vec() accessor
  REQUIRE(q4.as_vec() == v4);
}

TEST_CASE("Quat arithmetic operations", "[quat-arithmetic]") {
  Quatf q1(1.0f, 2.0f, 3.0f, 4.0f);
  Quatf q2(5.0f, 6.0f, 7.0f, 8.0f);

  // Addition
  auto sum = q1 + q2;
  REQUIRE(sum.w() == 6.0f);
  REQUIRE(sum.x() == 8.0f);
  REQUIRE(sum.y() == 10.0f);
  REQUIRE(sum.z() == 12.0f);

  // Subtraction
  auto diff = q2 - q1;
  REQUIRE(diff.w() == 4.0f);
  REQUIRE(diff.x() == 4.0f);
  REQUIRE(diff.y() == 4.0f);
  REQUIRE(diff.z() == 4.0f);

  // Scalar multiplication
  auto scaled = q1 * 2.0f;
  REQUIRE(scaled.w() == 2.0f);
  REQUIRE(scaled.x() == 4.0f);
  REQUIRE(scaled.y() == 6.0f);
  REQUIRE(scaled.z() == 8.0f);

  // Left scalar multiplication
  auto scaled2 = 3.0f * q1;
  REQUIRE(scaled2.w() == 3.0f);
  REQUIRE(scaled2.x() == 6.0f);
  REQUIRE(scaled2.y() == 9.0f);
  REQUIRE(scaled2.z() == 12.0f);
}

TEST_CASE("Quat Hamilton product", "[quat-hamilton]") {
  // Test with basis vectors
  Quatf i(0, 1, 0, 0);
  Quatf j(0, 0, 1, 0);
  Quatf k(0, 0, 0, 1);

  // i*i = -1
  auto ii = i * i;
  REQUIRE(ii.w() == Catch::Approx(-1.0f));
  REQUIRE(ii.x() == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(ii.y() == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(ii.z() == Catch::Approx(0.0f).margin(1e-6));

  // i*j = k
  auto ij = i * j;
  REQUIRE(ij.w() == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(ij.x() == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(ij.y() == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(ij.z() == Catch::Approx(1.0f));

  // j*i = -k
  auto ji = j * i;
  REQUIRE(ji.w() == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(ji.x() == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(ji.y() == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(ji.z() == Catch::Approx(-1.0f));

  Quatf q1(1, 2, 3, 4);
  Quatf q2(5, 6, 7, 8);

  // Manual calculation:
  // w = 1*5 - 2*6 - 3*7 - 4*8 = 5 - 12 - 21 - 32 = -60
  // x = 1*6 + 2*5 + 3*8 - 4*7 = 6 + 10 + 24 - 28 = 12
  // y = 1*7 - 2*8 + 3*5 + 4*6 = 7 - 16 + 15 + 24 = 30
  // z = 1*8 + 2*7 - 3*6 + 4*5 = 8 + 14 - 18 + 20 = 24
  auto prod = q1 * q2;
  REQUIRE(prod.w() == Catch::Approx(-60.0f));
  REQUIRE(prod.x() == Catch::Approx(12.0f));
  REQUIRE(prod.y() == Catch::Approx(30.0f));
  REQUIRE(prod.z() == Catch::Approx(24.0f));
}

TEST_CASE("Quat methods", "[quat-methods]") {
  Quatf q(1.0f, 2.0f, 3.0f, 4.0f);

  // norm_sqr
  // 1*1 + 2*2 + 3*3 + 4*4 = 1 + 4 + 9 + 16 = 30
  REQUIRE(q.norm_sqr() == Catch::Approx(30.0f));

  // norm
  REQUIRE(q.norm() == Catch::Approx(std::sqrt(30.0f)));

  // conjugate
  auto conj = q.conjugate();
  REQUIRE(conj.w() == 1.0f);
  REQUIRE(conj.x() == -2.0f);
  REQUIRE(conj.y() == -3.0f);
  REQUIRE(conj.z() == -4.0f);

  // inverse
  auto inv = q.inverse();
  // inv = conj / norm_sqr
  REQUIRE(inv.w() == Catch::Approx(1.0f / 30.0f));
  REQUIRE(inv.x() == Catch::Approx(-2.0f / 30.0f));
  REQUIRE(inv.y() == Catch::Approx(-3.0f / 30.0f));
  REQUIRE(inv.z() == Catch::Approx(-4.0f / 30.0f));

  // q * inv should be identity
  auto identity = q * inv;
  REQUIRE(identity.w() == Catch::Approx(1.0f));
  REQUIRE(identity.x() == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(identity.y() == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(identity.z() == Catch::Approx(0.0f).margin(1e-6));
}

TEST_CASE("UnitQuat construction and normalization",
          "[unitquat-construction]") {
  // From axis-angle
  Vec3f axis = {0, 0, 1};
  float angle = M_PI / 2.0f;  // 90 degrees
  UnitQuatf uq1(axis, angle);

  float half_angle = angle / 2.0f;  // PI / 4
  REQUIRE(uq1.w() == Catch::Approx(std::cos(half_angle)));
  REQUIRE(uq1.x() == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(uq1.y() == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(uq1.z() == Catch::Approx(std::sin(half_angle)));

  // norm should be 1
  Quatf q_uq1(uq1.w(), uq1.x(), uq1.y(), uq1.z());
  REQUIRE(q_uq1.norm() == Catch::Approx(1.0f));

  // From non-unit Quat
  Quatf q_non_unit(1, 1, 1, 1);
  UnitQuatf uq2(q_non_unit);
  Quatf q_uq2(uq2.w(), uq2.x(), uq2.y(), uq2.z());
  REQUIRE(q_uq2.norm() == Catch::Approx(1.0f));

  // Via Quat::normalized()
  UnitQuatf uq3 = q_non_unit.normalized();
  Quatf q_uq3(uq3.w(), uq3.x(), uq3.y(), uq3.z());
  REQUIRE(q_uq3.norm() == Catch::Approx(1.0f));
}

TEST_CASE("UnitQuat vector rotation", "[unitquat-rotation]") {
  SECTION("Rotate 90 degrees around Z axis") {
    Vec3f axis = {0, 0, 1};
    float angle = M_PI / 2.0f;  // 90 degrees
    UnitQuatf rot_z_90(axis, angle);

    Vec3f v_x = {1, 0, 0};
    Vec3f rotated_v = rot_z_90.rotate(v_x);

    // Rotating {1,0,0} by 90 deg around Z should give {0,1,0}
    REQUIRE(rotated_v.x() == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(rotated_v.y() == Catch::Approx(1.0f));
    REQUIRE(rotated_v.z() == Catch::Approx(0.0f).margin(1e-6));
  }

  SECTION("Rotate 90 degrees around Y axis") {
    Vec3f axis = {0, 1, 0};
    float angle = M_PI / 2.0f;  // 90 degrees
    UnitQuatf rot_y_90(axis, angle);

    Vec3f v_x = {1, 0, 0};
    Vec3f rotated_v = rot_y_90.rotate(v_x);

    // Rotating {1,0,0} by 90 deg around Y should give {0,0,-1}
    REQUIRE(rotated_v.x() == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(rotated_v.y() == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(rotated_v.z() == Catch::Approx(-1.0f));
  }

  SECTION("Rotate 180 degrees around Y axis") {
    Vec3f axis = {0, 1, 0};
    float angle = M_PI;  // 180 degrees
    UnitQuatf rot_y_180(axis, angle);

    Vec3f v_x = {1, 0, 0};
    Vec3f rotated_v = rot_y_180.rotate(v_x);

    // Rotating {1,0,0} by 180 deg around Y should give {-1,0,0}
    REQUIRE(rotated_v.x() == Catch::Approx(-1.0f));
    REQUIRE(rotated_v.y() == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(rotated_v.z() == Catch::Approx(0.0f).margin(1e-6));
  }
}

TEST_CASE("UnitQuat rotation composition (multiplication)",
          "[unitquat-composition]") {
  // 90 deg around Z, then 90 deg around Y
  UnitQuatf rot_z_90({0, 0, 1}, M_PI / 2.0f);
  UnitQuatf rot_y_90({0, 1, 0}, M_PI / 2.0f);

  // IMPORTANT: Quaternion multiplication order is opposite of matrix
  // multiplication for transformations To apply rot_z then rot_y, you do rot_y
  // * rot_z
  UnitQuatf combined = rot_y_90 * rot_z_90;

  Vec3f v = {1, 0, 0};

  // Apply separately
  Vec3f v_after_z = rot_z_90.rotate(v);  // -> {0, 1, 0}
  Vec3f v_final_sep = rot_y_90.rotate(
      v_after_z);  // rotating {0,1,0} around y-axis does nothing

  // Apply with combined quaternion
  Vec3f v_final_comb = combined.rotate(v);

  REQUIRE(v_final_sep.x() == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(v_final_sep.y() == Catch::Approx(1.0f));
  REQUIRE(v_final_sep.z() == Catch::Approx(0.0f).margin(1e-6));

  REQUIRE(v_final_comb.x() == Catch::Approx(v_final_sep.x()).margin(1e-7));
  REQUIRE(v_final_comb.y() == Catch::Approx(v_final_sep.y()).margin(1e-7));
  REQUIRE(v_final_comb.z() == Catch::Approx(v_final_sep.z()).margin(1e-7));
}

TEST_CASE("UnitQuat to_rotation_matrix", "[unitquat-tomatrix]") {
  Vec3f axis = {0, 0, 1};
  float angle = M_PI / 2.0f;  // 90 degrees
  UnitQuatf uq(axis, angle);

  Mat3f rot_mat = uq.to_rotation_matrix();

  // Check matrix is correct for a column-major library:
  // Rotated X-axis is the first column: {0, 1, 0}
  // Rotated Y-axis is the second column: {-1, 0, 0}
  // Rotated Z-axis is the third column: {0, 0, 1}
  // Resulting matrix:
  // [ 0 -1  0 ]
  // [ 1  0  0 ]
  // [ 0  0  1 ]

  REQUIRE(rot_mat(0, 0) == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(rot_mat(1, 0) == Catch::Approx(1.0f));
  REQUIRE(rot_mat(2, 0) == Catch::Approx(0.0f).margin(1e-6));

  REQUIRE(rot_mat(0, 1) == Catch::Approx(-1.0f));
  REQUIRE(rot_mat(1, 1) == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(rot_mat(2, 1) == Catch::Approx(0.0f).margin(1e-6));

  REQUIRE(rot_mat(0, 2) == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(rot_mat(1, 2) == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(rot_mat(2, 2) == Catch::Approx(1.0f));

  // Test if matrix rotation matches quat rotation
  Vec3f v = {1, 0, 0};
  Vec3f rotated_by_quat = uq.rotate(v);
  Vec3f rotated_by_mat = rot_mat * v;

  REQUIRE(rotated_by_mat.x() == Catch::Approx(rotated_by_quat.x()));
  REQUIRE(rotated_by_mat.y() == Catch::Approx(rotated_by_quat.y()));
  REQUIRE(rotated_by_mat.z() == Catch::Approx(rotated_by_quat.z()));
}

TEST_CASE("UnitQuat SLERP", "[unitquat-slerp]") {
  UnitQuatf id;  // Identity
  UnitQuatf rot_z_90({0, 0, 1}, M_PI / 2.0f);

  SECTION("t=0 returns start quaternion") {
    UnitQuatf result = UnitQuatf::slerp(id, rot_z_90, 0.0f);
    REQUIRE(result.w() == Catch::Approx(id.w()));
    REQUIRE(result.x() == Catch::Approx(id.x()));
    REQUIRE(result.y() == Catch::Approx(id.y()));
    REQUIRE(result.z() == Catch::Approx(id.z()));
  }

  SECTION("t=1 returns end quaternion") {
    UnitQuatf result = UnitQuatf::slerp(id, rot_z_90, 1.0f);
    REQUIRE(result.w() == Catch::Approx(rot_z_90.w()));
    REQUIRE(result.x() == Catch::Approx(rot_z_90.x()));
    REQUIRE(result.y() == Catch::Approx(rot_z_90.y()));
    REQUIRE(result.z() == Catch::Approx(rot_z_90.z()));
  }

  SECTION("t=0.5 returns halfway rotation") {
    UnitQuatf result = UnitQuatf::slerp(id, rot_z_90, 0.5f);
    UnitQuatf rot_z_45({0, 0, 1}, M_PI / 4.0f);  // 45 degrees

    REQUIRE(result.w() == Catch::Approx(rot_z_45.w()).margin(1e-7));
    REQUIRE(result.x() == Catch::Approx(rot_z_45.x()).margin(1e-7));
    REQUIRE(result.y() == Catch::Approx(rot_z_45.y()).margin(1e-7));
    REQUIRE(result.z() == Catch::Approx(rot_z_45.z()).margin(1e-7));
  }

  SECTION("Slerp takes the shorter path") {
    // q and -q represent the same rotation, but are on opposite sides of the
    // hypersphere
    UnitQuatf q1({0, 0, 1}, M_PI / 4.0);  // 45 deg
    Quatf q2_raw(q1.w(), q1.x(), q1.y(), q1.z());
    q2_raw = q2_raw * -1.0f;
    UnitQuatf q2(q2_raw);  // Represents the same 45 deg rotation, but its data
                           // is negated.

    // slerp should interpolate through an angle of 0, not a large angle
    UnitQuatf result = UnitQuatf::slerp(q1, q2, 0.5f);

    // The result should be close to q1 (or q2), as it's halfway on the short
    // arc Since the dot product was negative, q2 was flipped to -q2 (which is
    // q1's data) for the interpolation So we are interpolating between q1 and
    // effectively q1. The result should be q1.
    REQUIRE(result.w() == Catch::Approx(q1.w()).margin(1e-7));
    REQUIRE(result.x() == Catch::Approx(q1.x()).margin(1e-7));
    REQUIRE(result.y() == Catch::Approx(q1.y()).margin(1e-7));
    REQUIRE(result.z() == Catch::Approx(q1.z()).margin(1e-7));
  }

  SECTION("Slerp with almost identical quaternions falls back to LERP") {
    UnitQuatf q1({0, 0, 1}, 0.0001);
    UnitQuatf q2({0, 0, 1}, 0.0002);

    UnitQuatf result = UnitQuatf::slerp(q1, q2, 0.5f);
    UnitQuatf expected = UnitQuatf({0, 0, 1}, 0.00015);

    REQUIRE(result.w() == Catch::Approx(expected.w()).margin(1e-9));
    REQUIRE(result.x() == Catch::Approx(expected.x()).margin(1e-9));
    REQUIRE(result.y() == Catch::Approx(expected.y()).margin(1e-9));
    REQUIRE(result.z() == Catch::Approx(expected.z()).margin(1e-9));
  }
}