#pragma once

#include <stb_image.h>

#include <string>

#include "../core/image.hpp"

namespace soft_renderer {
namespace io {

static inline bool load_image_rgba8(const std::string& filename,
                                    core::ImageRGBA8& out) {
  int w = 0, h = 0, n = 0;
  stbi_uc* data = stbi_load(filename.c_str(), &w, &h, &n, 4);
  if (!data) return false;
  out = core::ImageRGBA8(static_cast<uint32_t>(w), static_cast<uint32_t>(h));
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      int idx = (y * w + x) * 4;
      core::RGBA8 c(data[idx + 0], data[idx + 1], data[idx + 2], data[idx + 3]);
      out(static_cast<uint32_t>(x), static_cast<uint32_t>(y)) = c;
    }
  }
  stbi_image_free(data);
  return true;
}

}  // namespace io
}  // namespace soft_renderer
