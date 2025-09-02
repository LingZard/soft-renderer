#pragma once

#include <concepts>

#include "../core/color.hpp"
#include "../core/math/mat.hpp"
#include "../core/math/vec.hpp"
#include "../core/texture.hpp"

namespace soft_renderer {
namespace renderer {

using namespace soft_renderer::core;

class FlatShader {
 public:
  struct Vertex {
    Vec4f position;
    Color color;
  };

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

class TexturedShader {
 public:
  struct Vertex {
    Vec4f position;
    Vec2f uv;
  };

  struct Uniforms {
    Mat4f mvp;
    const TextureRGBA8* texture = nullptr;
    SamplerState sampler;
  };

  struct Varyings {
    Vec4f clip_pos;
    Vec2f uv;

    Varyings operator+(const Varyings& other) const {
      return {.clip_pos = clip_pos + other.clip_pos, .uv = uv + other.uv};
    }

    Varyings operator-(const Varyings& other) const {
      return {.clip_pos = clip_pos - other.clip_pos, .uv = uv - other.uv};
    }

    Varyings operator*(float scalar) const {
      return {.clip_pos = clip_pos * scalar, .uv = uv * scalar};
    }
  };

  Varyings vertex(const Vertex& v, const Uniforms& u) const {
    return {
        .clip_pos = u.mvp * v.position,
        .uv = v.uv,
    };
  };

  Color fragment(const Varyings& vary, const Uniforms& u) const {
    if (u.texture == nullptr || u.texture->empty()) {
      return {1.0f, 0.0f, 1.0f, 1.0f};
    }
    RGBA8 c = u.texture->sample_uv(vary.uv.x(), vary.uv.y(), u.sampler);
    return rgba8_to_linear_color(c);
  };
};

}  // namespace renderer
}  // namespace soft_renderer