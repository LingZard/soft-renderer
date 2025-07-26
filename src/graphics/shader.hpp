#pragma once

#include <concepts>

#include "../core/color.hpp"
#include "../core/vertex.hpp"
#include "../math/mat.hpp"
#include "../math/vec.hpp"

namespace soft_renderer {
namespace graphics {

using namespace soft_renderer::core;

class FlatShader {
 public:
  struct Uniforms {
    Mat4f mvp;
  };

  struct Varyings {
    Vec4f clip_pos;
    Color color;

    Varyings operator+(const Varyings& other) const {
      return {.clip_pos = clip_pos + other.clip_pos,
              .color = color + other.color};
    }

    Varyings operator-(const Varyings& other) const {
      return {.clip_pos = clip_pos - other.clip_pos,
              .color = color - other.color};
    }

    Varyings operator*(float scalar) const {
      return {.clip_pos = clip_pos * scalar, .color = color * scalar};
    }
  };

  Varyings vertex(const Vertex& v, const Uniforms& u) const {
    return {
        .clip_pos = u.mvp * v.position,
        .color = v.color,
    };
  };

  Color fragment(const Varyings& vary, const Uniforms& u) const {
    return vary.color;
  };
};

}  // namespace graphics
}  // namespace soft_renderer