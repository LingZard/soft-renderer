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

// -------- Output transform (exposure + tone mapping + sRGB) --------

enum class OutputCurve { LinearToSRGB, Reinhard, ACES };

struct OutputSettings {
  OutputCurve curve = OutputCurve::LinearToSRGB;
  float exposure_ev = 0.0f;  // EV, scale = 2^EV
};

inline OutputSettings& global_output_settings() {
  static OutputSettings s;
  return s;
}

inline void set_output_settings(const OutputSettings& s) {
  global_output_settings() = s;
}

inline OutputSettings get_output_settings() { return global_output_settings(); }

inline Color apply_exposure(const Color& c, float exposure_ev) {
  float m = std::exp2(exposure_ev);
  return {c.x() * m, c.y() * m, c.z() * m, c.w()};
}

inline Color tonemap_reinhard(const Color& c) {
  auto tm = [](float x) { return x / (1.0f + x); };
  return {tm(std::max(0.0f, c.x())), tm(std::max(0.0f, c.y())),
          tm(std::max(0.0f, c.z())), c.w()};
}

// ACES approximation (Narkowicz 2015, "ACES Filmic Tone Mapping Curve")
inline Color tonemap_aces(const Color& c) {
  auto aces = [](float x) {
    // Clamp negative, apply curve
    x = std::max(0.0f, x);
    const float a = 2.51f;
    const float b = 0.03f;
    const float d = 2.43f;
    const float e = 0.59f;
    const float f = 0.14f;
    float num = x * (a * x + b);
    float den = x * (d * x + e) + f;
    return std::clamp(num / den, 0.0f, 1.0f);
  };
  return {aces(c.x()), aces(c.y()), aces(c.z()), c.w()};
}

inline RGBA8 linear_color_to_rgba8_output(const Color& linear_color) {
  // Apply exposure and tone mapping according to global settings, then sRGB
  OutputSettings s = get_output_settings();
  Color hdr = apply_exposure(linear_color, s.exposure_ev);
  Color ldr;
  switch (s.curve) {
    case OutputCurve::LinearToSRGB:
      ldr = hdr;
      break;
    case OutputCurve::Reinhard:
      ldr = tonemap_reinhard(hdr);
      break;
    case OutputCurve::ACES:
      ldr = tonemap_aces(hdr);
      break;
  }
  return linear_color_to_rgba8_srgb_exact(ldr);
}

}  // namespace core
}  // namespace soft_renderer