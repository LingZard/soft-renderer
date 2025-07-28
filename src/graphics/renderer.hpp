#pragma once
#include <memory>
#include <vector>

#include "../core/framebuffer.hpp"
#include "../core/primitive.hpp"
#include "clipping.hpp"
#include "pipeline_types.hpp"
#include "rasterizer.hpp"
#include "viewport.hpp"

namespace soft_renderer {
namespace graphics {

using namespace soft_renderer::core;

template <Shader TShader>
class Renderer {
 public:
  using Vertex = typename TShader::Vertex;
  using Varyings = typename TShader::Varyings;
  using Uniforms = typename TShader::Uniforms;

  Renderer(FrameBuffer& fb, const TShader& shader)
      : framebuffer_(fb), shader_(shader) {
    line_clipper_ =
        create_line_clipper<Varyings>(LineClipAlgorithm::LiangBarsky);
    triangle_clipper_ = create_triangle_clipper<Varyings>(
        TriangleClipAlgorithm::SutherlandHodgman);
  }

  void draw(const std::vector<Vertex>& vertices,
            const std::vector<uint32_t>& indices, const Uniforms& uniforms,
            PrimitiveTopology topology) {
    ViewportTransform fullscreen_viewport(framebuffer_.width(),
                                          framebuffer_.height());
    draw(vertices, indices, uniforms, topology, fullscreen_viewport);
  }

  void draw(const std::vector<Vertex>& vertices,
            const std::vector<uint32_t>& indices, const Uniforms& uniforms,
            PrimitiveTopology topology, const ViewportTransform& viewport) {
    // Vertex Processing
    std::vector<Varyings> varyings;
    varyings.reserve(vertices.size());
    for (const auto& vertex : vertices) {
      varyings.push_back(shader_.vertex(vertex, uniforms));
    }

    // Primitive Assembly & Rasterization & fragment processing
    switch (topology) {
      case PrimitiveTopology::Points:
        process_points(varyings, indices, uniforms, viewport);
        break;
      case PrimitiveTopology::Lines:
        process_lines(varyings, indices, uniforms, viewport);
        break;
      case PrimitiveTopology::Triangles:
        process_triangles(varyings, indices, uniforms, viewport);
        break;
    }
  }

 private:
  FrameBuffer& framebuffer_;
  const TShader& shader_;
  std::unique_ptr<ILineClipper<Varyings>> line_clipper_;
  std::unique_ptr<ITriangleClipper<Varyings>> triangle_clipper_;
  Rasterizer<TShader> rasterizer_;

  void process_points(const std::vector<Varyings>& varyings,
                      const std::vector<uint32_t>& indices,
                      const Uniforms& uniforms,
                      const ViewportTransform& viewport) {
    for (const auto& index : indices) {
      const auto& vary = varyings[index];
      // TODO: clipping

      // perspective divide
      float inv_w = 1.0f / vary.clip_pos.w();
      Vec3f ndc = vary.clip_pos.xyz() * inv_w;

      ScreenCoord screen_coord = viewport.ndc_to_screen(ndc);

      if (framebuffer_.depth_test(screen_coord.x, screen_coord.y,
                                  screen_coord.depth)) {
        Color final_color = shader_.fragment(vary, uniforms);
        RGBA8 color = to_rgba8(final_color);
        framebuffer_.set_pixel(screen_coord.x, screen_coord.y, color,
                               screen_coord.depth, false);
      }
    }
  }

  void process_lines(const std::vector<Varyings>& varyings,
                     const std::vector<uint32_t>& indices,
                     const Uniforms& uniforms,
                     const ViewportTransform& viewport) {
    for (size_t i = 0; i < indices.size(); i += 2) {
      const auto& v0 = varyings[indices[i]];
      const auto& v1 = varyings[indices[i + 1]];

      // 1. Line Clipping
      std::optional<std::pair<Varyings, Varyings>> clipped_lines =
          line_clipper_->clip(v0, v1);
      if (!clipped_lines) {
        continue;
      }

      const auto& [clipped_v0, clipped_v1] = clipped_lines.value();

      // 2. Perform perspective division and viewport transformation explicitly
      //    in the renderer.
      float inv_w0 = 1.0f / clipped_v0.clip_pos.w();
      Vec3f ndc0 = clipped_v0.clip_pos.xyz() * inv_w0;
      Fragment<TShader> sv0 = {viewport.ndc_to_screen(ndc0), clipped_v0};

      float inv_w1 = 1.0f / clipped_v1.clip_pos.w();
      Vec3f ndc1 = clipped_v1.clip_pos.xyz() * inv_w1;
      Fragment<TShader> sv1 = {viewport.ndc_to_screen(ndc1), clipped_v1};

      // 3. Hand over to rasterizer with a fragment processing lambda
      rasterizer_.rasterize_line(sv0, sv1, [&](const Fragment<TShader>& frag) {
        Color final_color = shader_.fragment(frag.varyings, uniforms);
        RGBA8 color = to_rgba8(final_color);
        framebuffer_.set_pixel(frag.screen_pos.x, frag.screen_pos.y, color,
                               frag.screen_pos.depth);
      });
    }
  }

  void process_triangles(const std::vector<Varyings>& varyings,
                         const std::vector<uint32_t>& indices,
                         const Uniforms& uniforms,
                         const ViewportTransform& viewport) {
    for (size_t i = 0; i < indices.size(); i += 3) {
      const auto& v0 = varyings[indices[i]];
      const auto& v1 = varyings[indices[i + 1]];
      const auto& v2 = varyings[indices[i + 2]];

      // 1. Back-face Culling (Optional optimization)
      // To perform back-face culling, we project the vertices to screen space
      // and check the winding order.
      Vec3f ndc0 = v0.clip_pos.xyz() * (1.0f / v0.clip_pos.w());
      ScreenCoord sc0 = viewport.ndc_to_screen(ndc0);
      Vec3f ndc1 = v1.clip_pos.xyz() * (1.0f / v1.clip_pos.w());
      ScreenCoord sc1 = viewport.ndc_to_screen(ndc1);
      Vec3f ndc2 = v2.clip_pos.xyz() * (1.0f / v2.clip_pos.w());
      ScreenCoord sc2 = viewport.ndc_to_screen(ndc2);

      float signed_area =
          (sc1.x - sc0.x) * (sc2.y - sc0.y) - (sc2.x - sc0.x) * (sc1.y - sc0.y);
      if (signed_area < 0) {
        continue;
      }

      // 2. Triangle Clipping
      std::vector<Varyings> clipped_vertices =
          triangle_clipper_->clip(v0, v1, v2);
      if (clipped_vertices.size() < 3) {
        continue;
      }

      // 3. Re-triangulate, transform, and rasterize
      const auto& anchor = clipped_vertices[0];
      float inv_w_anchor = 1.0f / anchor.clip_pos.w();
      Vec3f ndc_anchor = anchor.clip_pos.xyz() * inv_w_anchor;
      Fragment<TShader> sv_anchor = {viewport.ndc_to_screen(ndc_anchor),
                                     anchor};

      for (size_t i = 1; i < clipped_vertices.size() - 1; ++i) {
        const auto& p1 = clipped_vertices[i];
        const auto& p2 = clipped_vertices[i + 1];

        float inv_w1 = 1.0f / p1.clip_pos.w();
        Vec3f ndc1 = p1.clip_pos.xyz() * inv_w1;
        Fragment<TShader> sv1 = {viewport.ndc_to_screen(ndc1), p1};

        float inv_w2 = 1.0f / p2.clip_pos.w();
        Vec3f ndc2 = p2.clip_pos.xyz() * inv_w2;
        Fragment<TShader> sv2 = {viewport.ndc_to_screen(ndc2), p2};

        rasterizer_.rasterize_triangle(
            sv_anchor, sv1, sv2, [&](const Fragment<TShader>& frag) {
              Color final_color = shader_.fragment(frag.varyings, uniforms);
              RGBA8 color = to_rgba8(final_color);
              if (framebuffer_.depth_test(frag.screen_pos.x, frag.screen_pos.y,
                                          frag.screen_pos.depth)) {
                framebuffer_.set_pixel(frag.screen_pos.x, frag.screen_pos.y,
                                       color, frag.screen_pos.depth, false);
              }
            });
      }
    }
  }
};

}  // namespace graphics
}  // namespace soft_renderer