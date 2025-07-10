#pragma once
#include <cstdint>

#include "../math/vec.hpp"

namespace soft_renderer {
namespace core {

using namespace soft_renderer::math;

struct Vertex {
  Vec4f position;
  Vec4f color;
  Vec2f uv;
  Vec3f normal;
  Vec3f tangent;
};

}  // namespace core
}  // namespace soft_renderer