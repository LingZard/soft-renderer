#pragma once
#include <cstdint>
#include <vector>

#include "color.hpp"

namespace soft_renderer {
namespace core {

template <typename PixelType>
class Image {
 private:
  std::vector<PixelType> data_;
  uint32_t width_ = 0, height_ = 0;

 public:
  Image() = default;

  // Constructor for a black/zero-initialized image.
  Image(uint32_t width, uint32_t height)
      : data_(width * height), width_(width), height_(height) {}

  // Constructor to fill the image with a specific value.
  Image(uint32_t width, uint32_t height, const PixelType& fill_value)
      : data_(width * height, fill_value), width_(width), height_(height) {}

  uint32_t width() const { return width_; }
  uint32_t height() const { return height_; }

  PixelType* data() { return data_.data(); }
  const PixelType* data() const { return data_.data(); }

  PixelType& operator()(uint32_t x, uint32_t y) {
    return data_[y * width_ + x];
  }
  const PixelType& operator()(uint32_t x, uint32_t y) const {
    return data_[y * width_ + x];
  }
};

using ImageRGBA8 = Image<RGBA8>;
using ImageDepth32F = Image<float>;
}  // namespace core
}  // namespace soft_renderer