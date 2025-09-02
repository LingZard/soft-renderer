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

inline float srgb_to_linear_component(float srgb) {
  float c = std::clamp(srgb, 0.0f, 1.0f);
  if (c <= 0.04045f) return c / 12.92f;
  return powf((c + 0.055f) / 1.055f, 2.4f);
}

inline float linear_to_srgb_component(float linear) {
  float c = std::clamp(linear, 0.0f, 1.0f);
  if (c <= 0.0031308f) return 12.92f * c;
  return 1.055f * powf(c, 1.0f / 2.4f) - 0.055f;
}

inline Color rgba8_to_linear_color(const RGBA8& c) {
  float r = srgb_to_linear_component(static_cast<float>(c.r) / 255.0f);
  float g = srgb_to_linear_component(static_cast<float>(c.g) / 255.0f);
  float b = srgb_to_linear_component(static_cast<float>(c.b) / 255.0f);
  float a = static_cast<float>(c.a) / 255.0f;
  return {r, g, b, a};
}

inline RGBA8 linear_color_to_rgba8_srgb_exact(const Color& linear_color) {
  auto enc = [](float x) -> uint8_t {
    float s = linear_to_srgb_component(x);
    int u = static_cast<int>(std::round(s * 255.0f));
    return static_cast<uint8_t>(std::clamp(u, 0, 255));
  };
  uint8_t r = enc(linear_color.x());
  uint8_t g = enc(linear_color.y());
  uint8_t b = enc(linear_color.z());
  uint8_t a = static_cast<uint8_t>(std::clamp(
      static_cast<int>(std::round(linear_color.w() * 255.0f)), 0, 255));
  return RGBA8(r, g, b, a);
}

}  // namespace core
}  // namespace soft_renderer