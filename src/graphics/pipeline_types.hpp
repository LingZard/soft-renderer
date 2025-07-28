#pragma once
#include "../core/color.hpp"
#include "../math/vec.hpp"

namespace soft_renderer {
namespace graphics {

using namespace soft_renderer::core;

template <typename T>
concept Varying = requires(const T& a, const T& b, float scalar) {
  { a.clip_pos } -> std::same_as<const math::Vec4f&>;
  { a + b } -> std::same_as<T>;
  { a - b } -> std::same_as<T>;
  { a * scalar } -> std::same_as<T>;
};

template <typename T>
concept Vertex = requires(const T& t) {
  { t.position } -> std::same_as<const math::Vec4f&>;
};

template <typename T>
concept Shader =
    requires(T t, const typename T::Vertex& v, const typename T::Uniforms& u,
             const typename T::Varyings& vary) {
      typename T::Vertex;
      typename T::Uniforms;
      typename T::Varyings;
      { t.vertex(v, u) } -> std::same_as<typename T::Varyings>;
      { t.fragment(vary, u) } -> std::same_as<Color>;
      requires Varying<typename T::Varyings>;
      requires Vertex<typename T::Vertex>;
    };

struct ScreenCoord {
  int x;
  int y;
  float depth;
};

template <Shader TShader>
struct Fragment {
  ScreenCoord screen_pos;
  typename TShader::Varyings varyings;
};

}  // namespace graphics
}  // namespace soft_renderer