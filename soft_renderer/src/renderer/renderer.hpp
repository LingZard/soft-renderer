#pragma once
#include <memory>
#include <vector>

#include "../core/framebuffer.hpp"
#include "../scene/primitive.hpp"
#include "clipping.hpp"
#include "pipeline_types.hpp"
#include "rasterizer.hpp"
#include "viewport.hpp"

namespace soft_renderer {
namespace renderer {

using namespace soft_renderer::core;
using namespace soft_renderer::scene;

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
    bool use_weighted_blend = false;
    if constexpr (requires { uniforms.material.alpha_mode; }) {
      use_weighted_blend =
          (uniforms.material.alpha_mode == TShader::Material::AlphaMode::Blend);
    }
    if (use_weighted_blend) ensure_accum_buffers();

    // Vertex Processing
    std::vector<Varyings> varyings;
    varyings.reserve(vertices.size());
    for (const auto& vertex : vertices) {
      varyings.push_back(shader_.vertex(vertex, uniforms));
    }

    // Primitive Assembly & Rasterization & fragment processing
    switch (topology) {
      case PrimitiveTopology::Points:
        process_points(varyings, indices, uniforms, viewport,
                       use_weighted_blend);
        break;
      case PrimitiveTopology::Lines:
        process_lines(varyings, indices, uniforms, viewport,
                      use_weighted_blend);
        break;
      case PrimitiveTopology::Triangles:
        process_triangles(varyings, indices, uniforms, viewport,
                          use_weighted_blend);
        break;
    }

    if (use_weighted_blend) composite_weighted_over_framebuffer();
  }

 private:
  FrameBuffer& framebuffer_;
  const TShader& shader_;
  std::unique_ptr<ILineClipper<Varyings>> line_clipper_;
  std::unique_ptr<ITriangleClipper<Varyings>> triangle_clipper_;
  Rasterizer<TShader> rasterizer_;

  // Weighted blended OIT buffers (allocated per draw if needed)
  std::vector<math::Vec3f> accum_rgb_;
  std::vector<float> accum_w_;
  std::vector<float> accum_revealage_;

  void ensure_accum_buffers() {
    size_t sz =
        static_cast<size_t>(framebuffer_.width()) * framebuffer_.height();
    if (accum_rgb_.size() != sz) {
      accum_rgb_.assign(sz, math::Vec3f(0.0f, 0.0f, 0.0f));
      accum_w_.assign(sz, 0.0f);
      accum_revealage_.assign(sz, 1.0f);
    } else {
      std::fill(accum_rgb_.begin(), accum_rgb_.end(),
                math::Vec3f(0.0f, 0.0f, 0.0f));
      std::fill(accum_w_.begin(), accum_w_.end(), 0.0f);
      std::fill(accum_revealage_.begin(), accum_revealage_.end(), 1.0f);
    }
  }

  void composite_weighted_over_framebuffer() {
    uint32_t w = framebuffer_.width();
    uint32_t h = framebuffer_.height();
    const auto& img = framebuffer_.color_buffer();
    for (uint32_t y = 0; y < h; ++y) {
      for (uint32_t x = 0; x < w; ++x) {
        size_t idx = static_cast<size_t>(y) * w + x;
        float weight = accum_w_[idx];
        float reveal = accum_revealage_[idx];
        if (weight <= 0.0f && reveal >= 0.9999f)
          continue;  // nothing accumulated

        // Read destination (opaque) as background in linear
        RGBA8 dst = img(x, y);
        math::Vec3f dst_lin(
            srgb_to_linear_component(static_cast<float>(dst.r) / 255.0f),
            srgb_to_linear_component(static_cast<float>(dst.g) / 255.0f),
            srgb_to_linear_component(static_cast<float>(dst.b) / 255.0f));

        math::Vec3f accum = accum_rgb_[idx];
        math::Vec3f blended;
        if (weight > 0.0f) {
          blended = accum * (1.0f / weight) + dst_lin * reveal;
        } else {
          blended = dst_lin * reveal;
        }
        Color out{blended.x(), blended.y(), blended.z(), 1.0f};
        RGBA8 out_px = linear_color_to_rgba8_output(out);
        framebuffer_.set_pixel(x, y, out_px, /*depth*/ 0.0f,
                               /*test_depth*/ false);
      }
    }
  }

  void process_points(const std::vector<Varyings>& varyings,
                      const std::vector<uint32_t>& indices,
                      const Uniforms& uniforms,
                      const ViewportTransform& viewport,
                      bool use_weighted_blend) {
    for (const auto& index : indices) {
      const auto& vary = varyings[index];
      // TODO: clipping

      // perspective divide
      float inv_w = 1.0f / vary.clip_pos.w();
      Vec3f ndc = vary.clip_pos.xyz() * inv_w;

      ScreenCoord screen_coord = viewport.ndc_to_screen(ndc);

      if (!use_weighted_blend) {
        if (framebuffer_.depth_test(screen_coord.x, screen_coord.y,
                                    screen_coord.depth)) {
          Color final_color = shader_.fragment(vary, uniforms);
          if (final_color.w() < 0.0f) {
            return;
          }
          RGBA8 color = linear_color_to_rgba8_output(final_color);
          framebuffer_.set_pixel(screen_coord.x, screen_coord.y, color,
                                 screen_coord.depth, false);
        }
      } else {
        Color final_color = shader_.fragment(vary, uniforms);
        if (final_color.w() < 0.0f) continue;
        float a = std::clamp(final_color.w(), 0.0f, 1.0f);
        float depth = std::clamp(screen_coord.depth, 0.0f, 1.0f);
        float w = a * std::max(1e-2f, std::pow(1.0f - depth, 4.0f));
        size_t idx =
            static_cast<size_t>(screen_coord.y) * framebuffer_.width() +
            screen_coord.x;
        accum_rgb_[idx] =
            accum_rgb_[idx] +
            math::Vec3f(final_color.x(), final_color.y(), final_color.z()) * w;
        accum_w_[idx] += w;
        accum_revealage_[idx] *= (1.0f - a);
      }
    }
  }

  void process_lines(const std::vector<Varyings>& varyings,
                     const std::vector<uint32_t>& indices,
                     const Uniforms& uniforms,
                     const ViewportTransform& viewport,
                     bool use_weighted_blend) {
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
        if (final_color.w() < 0.0f) return;  // alpha test discard
        if (!use_weighted_blend) {
          RGBA8 color = linear_color_to_rgba8_output(final_color);
          framebuffer_.set_pixel(frag.screen_pos.x, frag.screen_pos.y, color,
                                 frag.screen_pos.depth);
        } else {
          float a = std::clamp(final_color.w(), 0.0f, 1.0f);
          float depth = std::clamp(frag.screen_pos.depth, 0.0f, 1.0f);
          float w = a * std::max(1e-2f, std::pow(1.0f - depth, 4.0f));
          size_t idx =
              static_cast<size_t>(frag.screen_pos.y) * framebuffer_.width() +
              frag.screen_pos.x;
          accum_rgb_[idx] =
              accum_rgb_[idx] +
              math::Vec3f(final_color.x(), final_color.y(), final_color.z()) *
                  w;
          accum_w_[idx] += w;
          accum_revealage_[idx] *= (1.0f - a);
        }
      });
    }
  }

  void process_triangles(const std::vector<Varyings>& varyings,
                         const std::vector<uint32_t>& indices,
                         const Uniforms& uniforms,
                         const ViewportTransform& viewport,
                         bool use_weighted_blend) {
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
      // skip back-face culling
      // if (signed_area < 0) {
      //   continue;
      // }

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
              if (final_color.w() < 0.0f) return;  // alpha test discard
              if (!use_weighted_blend) {
                RGBA8 color = linear_color_to_rgba8_output(final_color);
                if (framebuffer_.depth_test(frag.screen_pos.x,
                                            frag.screen_pos.y,
                                            frag.screen_pos.depth)) {
                  framebuffer_.set_pixel(frag.screen_pos.x, frag.screen_pos.y,
                                         color, frag.screen_pos.depth, false);
                }
              } else {
                float a = std::clamp(final_color.w(), 0.0f, 1.0f);
                float depth = std::clamp(frag.screen_pos.depth, 0.0f, 1.0f);
                float w = a * std::max(1e-2f, std::pow(1.0f - depth, 4.0f));
                size_t idx = static_cast<size_t>(frag.screen_pos.y) *
                                 framebuffer_.width() +
                             frag.screen_pos.x;
                accum_rgb_[idx] = accum_rgb_[idx] +
                                  math::Vec3f(final_color.x(), final_color.y(),
                                              final_color.z()) *
                                      w;
                accum_w_[idx] += w;
                accum_revealage_[idx] *= (1.0f - a);
              }
            });
      }
    }
  }
};

}  // namespace renderer
}  // namespace soft_renderer