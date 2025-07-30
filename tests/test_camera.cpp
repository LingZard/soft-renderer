#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>

#include "soft_renderer/camera.hpp"
#include "soft_renderer/core.hpp"

using namespace soft_renderer::camera;
using namespace soft_renderer::math;

TEST_CASE("Camera View Matrix", "[camera-view]") {
  SECTION("Camera at origin with no rotation") {
    // The camera stores a CV-style pose (Y-down, Z-out).
    // The view matrix transforms to an OpenGL view space (Y-up, Z-in).
    // This requires a 180-degree rotation around the X-axis.
    PerspectiveCamera cam({0, 0, 0}, UnitQuatd::identity(), M_PI_2);
    Mat4d view = cam.get_view_matrix();

    Mat4d expected_cv_to_gl = Mat4d::identity();
    expected_cv_to_gl(1, 1) = -1.0;
    expected_cv_to_gl(2, 2) = -1.0;

    REQUIRE(view == expected_cv_to_gl);
  }

  SECTION("Translated and rotated camera correctly transforms its position") {
    Vec3d position = {10.0, -20.0, 30.0};
    UnitQuatd orientation = UnitQuatd::from_axis_angle(
        {0.0, 1.0, 0.0}, M_PI / 2.0);  // 90-degree yaw
    PerspectiveCamera cam(position, orientation, 1.0);

    Mat4d view = cam.get_view_matrix();

    // Key invariant: The view matrix must transform the camera's own world
    // position to the origin of the view space.
    Vec4d transformed_pos = view * Vec4d(position, 1.0);
    REQUIRE(transformed_pos.x() == Catch::Approx(0.0).margin(1e-9));
    REQUIRE(transformed_pos.y() == Catch::Approx(0.0).margin(1e-9));
    REQUIRE(transformed_pos.z() == Catch::Approx(0.0).margin(1e-9));
    REQUIRE(transformed_pos.w() == Catch::Approx(1.0).margin(1e-9));
  }
}

TEST_CASE("Camera Projection Matrix Generation", "[camera-projection]") {
  SECTION("PerspectiveCamera forwards parameters correctly") {
    PerspectiveCamera cam({0, 0, 0}, UnitQuatd::identity(), M_PI / 2.0);
    double aspect = 16.0 / 9.0;
    double near = 0.1;
    double far = 100.0;
    Mat4d proj = cam.get_projection_matrix(aspect, near, far);
    Mat4d expected = perspective(aspect, M_PI / 2.0, near, far);
    REQUIRE(proj == expected);
  }

  SECTION("OrthographicCamera forwards parameters correctly") {
    double left = -10, right = 10, bottom = -5, top = 5;
    double near = 0.1, far = 100.0;
    OrthographicCamera cam({0, 0, 0}, UnitQuatd::identity(), left, right,
                           bottom, top);
    Mat4d proj = cam.get_projection_matrix(1.0, near, far);  // aspect is unused
    Mat4d expected = orthographic(left, right, bottom, top, near, far);
    REQUIRE(proj == expected);
  }
}