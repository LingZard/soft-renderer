#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>
#include <type_traits>

#include "../src/math/vec.hpp"

using namespace soft_renderer::math;

TEST_CASE("Vec2 basic operations", "[vec2]") {
  Vec<float, 2> v1{1.0f, 2.0f};
  Vec<float, 2> v2{3.0f, 4.0f};
  REQUIRE(v1.x() == 1.0f);
  REQUIRE(v1.y() == 2.0f);
  REQUIRE((v1 + v2)[0] == 4.0f);
  REQUIRE((v1 + v2)[1] == 6.0f);
  REQUIRE((v2 - v1)[0] == 2.0f);
  REQUIRE((v2 - v1)[1] == 2.0f);
  REQUIRE((v1 * 2.0f)[0] == 2.0f);
  REQUIRE((v1 * 2.0f)[1] == 4.0f);
  REQUIRE(v1.dot(v2) == Catch::Approx(11.0f));
  REQUIRE(v1.size() == 2);
}

TEST_CASE("Vec3 cross and norm", "[vec3]") {
  Vec<float, 3> a{1.0f, 0.0f, 0.0f};
  Vec<float, 3> b{0.0f, 1.0f, 0.0f};
  auto c = a.cross(b);
  REQUIRE(c.x() == Catch::Approx(0.0f));
  REQUIRE(c.y() == Catch::Approx(0.0f));
  REQUIRE(c.z() == Catch::Approx(1.0f));
  REQUIRE(a.norm() == Catch::Approx(1.0f));
  REQUIRE(b.norm() == Catch::Approx(1.0f));
  REQUIRE(c.norm() == Catch::Approx(1.0f));
}

TEST_CASE("Vec4 access and arithmetic", "[vec4]") {
  Vec<float, 4> v{1, 2, 3, 4};
  REQUIRE(v.x() == 1);
  REQUIRE(v.y() == 2);
  REQUIRE(v.z() == 3);
  REQUIRE(v.w() == 4);
  auto v2 = v * 2.0f;
  REQUIRE(v2.x() == 2);
  REQUIRE(v2.y() == 4);
  REQUIRE(v2.z() == 6);
  REQUIRE(v2.w() == 8);
}

TEST_CASE("VecBase zero/ones/normalize", "[vecbase]") {
  auto v = Vec<float, 3>::zero();
  REQUIRE(v.x() == 0.0f);
  REQUIRE(v.y() == 0.0f);
  REQUIRE(v.z() == 0.0f);
  auto o = Vec<float, 3>::ones();
  REQUIRE(o.x() == 1.0f);
  REQUIRE(o.y() == 1.0f);
  REQUIRE(o.z() == 1.0f);
  Vec<float, 3> n{3.0f, 0.0f, 4.0f};
  n.normalize();
  REQUIRE(n.norm() == Catch::Approx(1.0f));
}

TEST_CASE("Vec equality and assignment", "[vec-eq]") {
  Vec<int, 2> a{1, 2};
  Vec<int, 2> b{1, 2};
  Vec<int, 2> c{2, 3};
  REQUIRE(a == b);
  REQUIRE_FALSE(a == c);
  a += c;
  REQUIRE(a[0] == 3);
  REQUIRE(a[1] == 5);
  a -= b;
  REQUIRE(a[0] == 2);
  REQUIRE(a[1] == 3);
  a *= 2;
  REQUIRE(a[0] == 4);
  REQUIRE(a[1] == 6);
}

TEST_CASE("Vec supports initializer_list and copy/move", "[vec-init]") {
  Vec<double, 3> v{1.0, 2.0, 3.0};
  Vec<double, 3> v2 = v;
  REQUIRE(v2.x() == 1.0);
  REQUIRE(v2.y() == 2.0);
  REQUIRE(v2.z() == 3.0);
  Vec<double, 3> v3 = std::move(v2);
  REQUIRE(v3.x() == 1.0);
  REQUIRE(v3.y() == 2.0);
  REQUIRE(v3.z() == 3.0);
}

TEST_CASE("Vec extended operations", "[vec-extended]") {
  Vec<float, 3> v1{1.0f, 2.0f, 3.0f};
  Vec<float, 3> v2{4.0f, 5.0f, 6.0f};

  // Test length_sqr()
  REQUIRE(v1.length_sqr() ==
          Catch::Approx(1.0f * 1.0f + 2.0f * 2.0f + 3.0f * 3.0f));
  REQUIRE(v2.length_sqr() ==
          Catch::Approx(4.0f * 4.0f + 5.0f * 5.0f + 6.0f * 6.0f));

  // Test component-wise multiplication
  auto v_mul = v1 * v2;
  REQUIRE(v_mul[0] == Catch::Approx(1.0f * 4.0f));
  REQUIRE(v_mul[1] == Catch::Approx(2.0f * 5.0f));
  REQUIRE(v_mul[2] == Catch::Approx(3.0f * 6.0f));

  // Test component-wise division
  auto v_div = v2 / v1;
  REQUIRE(v_div[0] == Catch::Approx(4.0f / 1.0f));
  REQUIRE(v_div[1] == Catch::Approx(5.0f / 2.0f));
  REQUIRE(v_div[2] == Catch::Approx(6.0f / 3.0f));

  // Test lerp()
  Vec<float, 2> a{0.0f, 0.0f};
  Vec<float, 2> b{10.0f, 10.0f};
  auto mid = lerp(a, b, 0.5f);
  REQUIRE(mid[0] == Catch::Approx(5.0f));
  REQUIRE(mid[1] == Catch::Approx(5.0f));

  auto quarter = lerp(a, b, 0.25f);
  REQUIRE(quarter[0] == Catch::Approx(2.5f));
  REQUIRE(quarter[1] == Catch::Approx(2.5f));
}
