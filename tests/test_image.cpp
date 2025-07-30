#include <catch2/catch_test_macros.hpp>
#include <cstdio>
#include <filesystem>
#include <string>

#include "soft_renderer/core.hpp"
#include "soft_renderer/io.hpp"

using namespace soft_renderer;

TEST_CASE("ColorTest", "[core]") {
  SECTION("RGBA8 Constructors") {
    core::RGBA8 color1(10, 20, 30, 40);
    REQUIRE(color1.r == 10);
    REQUIRE(color1.g == 20);
    REQUIRE(color1.b == 30);
    REQUIRE(color1.a == 40);

    core::RGBA8 color2(50, 60, 70);
    REQUIRE(color2.r == 50);
    REQUIRE(color2.g == 60);
    REQUIRE(color2.b == 70);
    REQUIRE(color2.a == 255);
  }
}

TEST_CASE("ImageTest", "[core]") {
  SECTION("Default Constructor") {
    core::Image<int> image;
    REQUIRE(image.width() == 0);
    REQUIRE(image.height() == 0);
    REQUIRE(image.data() == nullptr);
  }

  SECTION("Sized Constructor") {
    core::Image<core::RGBA8> image(100, 200);
    REQUIRE(image.width() == 100);
    REQUIRE(image.height() == 200);
    REQUIRE(image.data() != nullptr);
    // Default constructor of RGBA8 is not defined, so value is undefined.
    // We just check if memory is allocated.
  }

  SECTION("Fill Constructor") {
    core::RGBA8 red(255, 0, 0);
    core::Image<core::RGBA8> image(50, 50, red);
    REQUIRE(image.width() == 50);
    REQUIRE(image.height() == 50);
    REQUIRE(image(25, 25).r == 255);
    REQUIRE(image(0, 0).g == 0);
  }

  SECTION("Pixel Access") {
    core::Image<core::RGBA8> image(10, 10);
    core::RGBA8 blue(0, 0, 255);
    image(5, 5) = blue;
    REQUIRE(image(5, 5).b == 255);
    REQUIRE(image(5, 5).r == 0);
    REQUIRE(image(5, 5).a == 255);
  }
}

TEST_CASE("TgaImageTest", "[io]") {
  const std::string test_filename = "test_image.tga";
  const uint32_t width = 10;
  const uint32_t height = 20;

  // Cleanup any old test file
  if (std::filesystem::exists(test_filename)) {
    std::remove(test_filename.c_str());
  }

  SECTION("Write and Read TGA") {
    // 1. Create a test image with distinct top and bottom halves
    core::Image<core::RGBA8> original_image(width, height);
    core::RGBA8 red(255, 0, 0);
    core::RGBA8 green(0, 255, 0);

    for (uint32_t y = 0; y < height / 2; ++y) {
      for (uint32_t x = 0; x < width; ++x) {
        original_image(x, y) = red;
      }
    }
    for (uint32_t y = height / 2; y < height; ++y) {
      for (uint32_t x = 0; x < width; ++x) {
        original_image(x, y) = green;
      }
    }

    // 2. Write the image to a TGA file
    bool success = io::write_tga_image(test_filename, original_image);
    REQUIRE(success);
    REQUIRE(std::filesystem::exists(test_filename));

    // 3. Read the image back
    core::Image<core::RGBA8> loaded_image = io::read_tga_image(test_filename);
    REQUIRE(loaded_image.width() == width);
    REQUIRE(loaded_image.height() == height);

    // 4. Compare the images pixel by pixel
    bool images_match = true;
    for (uint32_t y = 0; y < height; ++y) {
      for (uint32_t x = 0; x < width; ++x) {
        if (original_image(x, y).r != loaded_image(x, y).r ||
            original_image(x, y).g != loaded_image(x, y).g ||
            original_image(x, y).b != loaded_image(x, y).b ||
            original_image(x, y).a != loaded_image(x, y).a) {
          images_match = false;
          break;
        }
      }
      if (!images_match) break;
    }

    // This check will likely fail if the origin is handled incorrectly.
    REQUIRE(images_match);
  }

  // Final cleanup
  if (std::filesystem::exists(test_filename)) {
    std::remove(test_filename.c_str());
  }
}