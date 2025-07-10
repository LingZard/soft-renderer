#pragma once

#include <concepts>

#include "../core/color.hpp"
#include "../core/vertex.hpp"
#include "../math/mat.hpp"
#include "../math/vec.hpp"

namespace soft_renderer {
namespace graphics {

using namespace soft_renderer::core;

template <typename T>
concept Shader = requires(T t, const Vertex& v, const typename T::Uniforms& u,
                          const typename T::Varyings& vary) {
  typename T::Uniforms;
  typename T::Varyings;
  { t.vertex(v, u) } -> std::same_as<typename T::Varyings>;
  { t.fragment(vary, u) } -> std::same_as<Color>;
};

class FlatShader {
 public:
  struct Uniforms {
    Mat4f mvp;
  };

  struct Varyings {
    Vec4f clip_pos;
    Color color;
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