#pragma once
#include <cstdint>

namespace soft_renderer {
namespace core {

// sRGB
struct RGBA8 {
  uint8_t r, g, b, a;

  RGBA8() : r(0), g(0), b(0), a(255) {}

  RGBA8(uint8_t r, uint8_t g, uint8_t b, uint8_t a) : r(r), g(g), b(b), a(a) {}

  RGBA8(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b), a(255) {}
};

}  // namespace core
}  // namespace soft_renderer