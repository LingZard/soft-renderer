#pragma once

#include "color.hpp"
#include "image.hpp"

namespace soft_renderer {
namespace core {

class FrameBuffer {
 public:
  // Creates a framebuffer with a specific width and height.
  // The color buffer is default-initialized (usually to black), and the depth
  // buffer is initialized with a value of 1.0f, representing the farthest
  // possible distance.
  FrameBuffer(uint32_t width, uint32_t height)
      : width_(width),
        height_(height),
        color_buffer_(width, height),
        depth_buffer_(width, height, 1.0f) {}

  void clear(const RGBA8& clear_color) {
    for (uint32_t y = 0; y < height_; ++y) {
      for (uint32_t x = 0; x < width_; ++x) {
        color_buffer_(x, y) = clear_color;
        depth_buffer_(x, y) = 1.0f;
      }
    }
  }

  bool depth_test(uint32_t x, uint32_t y, float depth) const {
    if (x >= width_ || y >= height_) {
      return false;
    }

    return depth < depth_buffer_(x, y);
  }

  void set_pixel(uint32_t x, uint32_t y, const RGBA8& color, float depth,
                 bool test_depth = true) {
    if (!test_depth || depth_test(x, y, depth)) {
      depth_buffer_(x, y) = depth;
      color_buffer_(x, y) = color;
    }
  }

  const ImageRGBA8& color_buffer() const { return color_buffer_; }

  uint32_t width() const { return width_; }
  uint32_t height() const { return height_; }

 private:
  uint32_t width_;
  uint32_t height_;
  ImageRGBA8 color_buffer_;
  ImageDepth32F depth_buffer_;
};

}  // namespace core
}  // namespace soft_renderer