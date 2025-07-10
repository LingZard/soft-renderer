#pragma once
#include <vector>

#include "../core/framebuffer.hpp"
#include "../core/primitive.hpp"
#include "../core/vertex.hpp"
#include "shader.hpp"

namespace soft_renderer {
namespace graphics {

using namespace soft_renderer::core;

template <Shader TShader>
class Renderer {
 public:
  using Varyings = typename TShader::Varyings;
  using Uniforms = typename TShader::Uniforms;

  Renderer(FrameBuffer& fb, const TShader& shader)
      : framebuffer_(fb), shader_(shader) {}

  void draw(const std::vector<Vertex>& vertices,
            const std::vector<uint32_t>& indices, const Uniforms& uniforms,
            PrimitiveTopology topology) {
    // Vertex Processing
    std::vector<Varyings> varyings;
    varyings.reserve(vertices.size());
    for (const auto& vertex : vertices) {
      varyings.push_back(shader_.vertex(vertex, uniforms));
    }

    // Primitive Assembly & Rasterization & fragment processing
    switch (topology) {
      case PrimitiveTopology::Points:
        process_points(varyings, indices, uniforms);
        break;
      case PrimitiveTopology::Lines:
        break;
      case PrimitiveTopology::Triangles:
        break;
    }
  }

 private:
  FrameBuffer& framebuffer_;
  const TShader& shader_;

  void process_points(const std::vector<Varyings>& varyings,
                      const std::vector<uint32_t>& indices,
                      const Uniforms& uniforms) {
    for (const auto& index : indices) {
      const auto& vary = varyings[index];
      // TODO: clipping

      // perspective divide
      float inv_w = 1.0f / vary.clip_pos.w();
      Vec3f ndc = vary.clip_pos.xyz() * inv_w;

      int screen_x = (ndc.x() + 1.0f) * 0.5f * framebuffer_.width();
      int screen_y = (ndc.y() + 1.0f) * 0.5f * framebuffer_.height();
      float depth = (ndc.z() + 1.0f) * 0.5f;

      if (framebuffer_.depth_test(screen_x, screen_y, depth)) {
        Color final_color = shader_.fragment(vary, uniforms);
        RGBA8 color = to_rgba8(final_color);
        framebuffer_.set_pixel(screen_x, screen_y, color, depth, false);
      }
    }
  }

  void process_lines(const std::vector<Varyings>& varyings,
                     const std::vector<uint32_t>& indices,
                     const Uniforms& uniforms) {
    for (size_t i = 0; i < indices.size(); i += 2) {
      const auto& v0 = varyings[indices[i]];
      const auto& v1 = varyings[indices[i + 1]];

      // TODO: 1. Line Clipping
      // Clip the line segment against the view frustum.
      // This is complex, may produce 0 or 1 new lines. E.g., using
      // Liang-Barsky.

      // TODO: 2. Perspective Divide & Viewport Transform for both vertices.

      // TODO: 3. Line Rasterization
      // Use an algorithm like Bresenham's to find all pixels between the two
      // endpoints. For each pixel (x, y) on the line:
      //   a. Interpolate attributes from v0 and v1 for the current pixel.
      //      This requires perspective-correct interpolation.
      //      - Calculate an interpolation factor `t`.
      //      - Interpolate `1/w` and `attribute/w`.
      //      - Final attribute = (interpolated attribute/w) / (interpolated
      //      1/w).
      //   b. Perform depth test with the interpolated depth.
      //   c. If the test passes, call fragment shader and write to framebuffer.
    }
  }

  void process_triangles(const std::vector<Varyings>& varyings,
                         const std::vector<uint32_t>& indices,
                         const Uniforms& uniforms) {
    for (size_t i = 0; i < indices.size(); i += 3) {
      const auto& v0 = varyings[indices[i]];
      const auto& v1 = varyings[indices[i + 1]];
      const auto& v2 = varyings[indices[i + 2]];

      // TODO: 1. Back-face Culling (Optional optimization)
      // Determine triangle orientation in screen space. If it's a back-face,
      // discard it. e.g., by checking the sign of the cross product of two
      // edges.

      // TODO: 2. Triangle Clipping
      // Clip the triangle against the view frustum. This is the most complex
      // part. A clipped triangle can become a polygon of 3 to 7 vertices. This
      // polygon must then be re-triangulated.

      // TODO: 3. Perspective Divide & Viewport Transform for all 3 vertices.

      // TODO: 4. Triangle Rasterization
      //   a. Compute a 2D bounding box for the triangle on the screen.
      //   b. Loop through each pixel (x, y) within the bounding box.
      //   c. Calculate barycentric coordinates (alpha, beta, gamma) for the
      //   pixel. d. If the pixel is inside the triangle (coordinates are all
      //   non-negative):
      //      i. Perform perspective-correct interpolation of attributes using
      //      the barycentric coordinates. ii. Perform depth test using the
      //      interpolated depth. iii. If the test passes, call fragment shader
      //      and write to framebuffer.
    }
  }
};

}  // namespace graphics
}  // namespace soft_renderer