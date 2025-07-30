#include <catch2/catch_test_macros.hpp>

#include "soft_renderer/core.hpp"

using namespace soft_renderer;
using namespace soft_renderer::core;

TEST_CASE("FramebufferTest", "[core]") {
  const uint32_t width = 10;
  const uint32_t height = 20;

  SECTION("Initialization and Clearing") {
    FrameBuffer fb(width, height);
    REQUIRE(fb.width() == width);
    REQUIRE(fb.height() == height);

    RGBA8 clear_color(10, 20, 30, 40);
    fb.clear(clear_color);

    // Check a few pixels to see if they have the clear color.
    const auto& color_buffer = fb.color_buffer();
    REQUIRE(color_buffer(0, 0).r == 10);
    REQUIRE(color_buffer(5, 5).g == 20);
    REQUIRE(color_buffer(9, 19).b == 30);
    REQUIRE(color_buffer(0, 0).a == 40);

    // We can't directly check the depth buffer, but we can infer its state
    // through the set_pixel behavior in the next test.
  }

  SECTION("Depth Test (Z-Buffering)") {
    FrameBuffer fb(width, height);
    RGBA8 black(0, 0, 0);
    fb.clear(black);  // Start with a black canvas and depth = 1.0

    RGBA8 red(255, 0, 0);
    RGBA8 green(0, 255, 0);
    RGBA8 blue(0, 0, 255);
    const uint32_t test_x = 5;
    const uint32_t test_y = 10;

    // 1. Draw a pixel that is far away.
    // Depth is in [0, 1], where 0 is near and 1 is far.
    fb.set_pixel(test_x, test_y, red, 0.8f);
    REQUIRE(fb.color_buffer()(test_x, test_y).r ==
            255);  // Red should be drawn.

    // 2. Draw a closer pixel at the same location.
    fb.set_pixel(test_x, test_y, green, 0.5f);
    // The green pixel is closer (0.5 < 0.8), so it should overwrite the red
    // one.
    REQUIRE(fb.color_buffer()(test_x, test_y).g == 255);
    REQUIRE(fb.color_buffer()(test_x, test_y).r == 0);

    // 3. Try to draw a pixel that is farther away than the current one.
    fb.set_pixel(test_x, test_y, blue, 0.7f);
    // The blue pixel is farther (0.7 > 0.5), so it should be discarded.
    // The pixel should still be green.
    REQUIRE(fb.color_buffer()(test_x, test_y).g == 255);
    REQUIRE(fb.color_buffer()(test_x, test_y).b == 0);

    // 4. Draw a pixel at the absolute nearest depth.
    fb.set_pixel(test_x, test_y, blue, 0.0f);
    REQUIRE(fb.color_buffer()(test_x, test_y).b == 255);
  }

  SECTION("Bounds Checking") {
    FrameBuffer fb(width, height);
    RGBA8 white(255, 255, 255);
    // These calls should not crash or corrupt memory.
    // We can't easily verify they did nothing, but a passing test
    // (i.e., no crash) is a good sign.
    fb.set_pixel(width, height, white, 0.5f);
    fb.set_pixel(width, 5, white, 0.5f);
    fb.set_pixel(5, height, white, 0.5f);
    fb.set_pixel(999, 999, white, 0.5f);

    // To be sure, let's clear the buffer to black and check a corner.
    // If the out-of-bounds writes did something evil, this might fail.
    fb.clear({0, 0, 0});
    REQUIRE(fb.color_buffer()(0, 0).r == 0);
    REQUIRE(fb.color_buffer()(width - 1, height - 1).r == 0);
  }
}