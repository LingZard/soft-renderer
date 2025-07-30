#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

#include "math/vec.hpp"

namespace soft_renderer {
namespace core {

using namespace soft_renderer::math;
using Color = Vec4f;

// sRGB
struct RGBA8 {
  uint8_t r, g, b, a;

  RGBA8() : r(0), g(0), b(0), a(255) {}

  RGBA8(uint8_t r, uint8_t g, uint8_t b, uint8_t a) : r(r), g(g), b(b), a(a) {}

  RGBA8(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b), a(255) {}
};

// Converts a linear floating-point color [0, 1] to a gamma-corrected 8-bit
// sRGB color [0, 255].
inline RGBA8 to_rgba8(const Color& linear_color) {
  auto gamma_correct = [](float c) {
    // Using approximate gamma 2.2 for sRGB conversion
    float clamped = std::max(0.0f, std::min(1.0f, c));
    return static_cast<uint8_t>(powf(clamped, 1.0f / 2.2f) * 255.0f);
  };

  uint8_t r = gamma_correct(linear_color.x());
  uint8_t g = gamma_correct(linear_color.y());
  uint8_t b = gamma_correct(linear_color.z());
  // Alpha channel is typically not gamma-corrected
  uint8_t a = static_cast<uint8_t>(
      std::max(0.0f, std::min(1.0f, linear_color.w())) * 255.0f);

  return RGBA8(r, g, b, a);
}

}  // namespace core
}  // namespace soft_renderer