#pragma once

#include <vector>

#include "../core/framebuffer.hpp"
#include "../core/texture.hpp"
#include "pipeline_types.hpp"
#include "viewport.hpp"

namespace soft_renderer {
namespace renderer {

using namespace soft_renderer::core;

template <Shader TShader>
class Rasterizer {
 public:
  using Varyings = typename TShader::Varyings;
  using Uniforms = typename TShader::Uniforms;

  void rasterize_triangle(
      const Fragment<TShader>& v0, const Fragment<TShader>& v1,
      const Fragment<TShader>& v2,
      std::function<void(const Fragment<TShader>&)> fragment_processor) {
    // 1. Get screen-space coordinates as 2D vectors
    math::Vec2i p0 = {v0.screen_pos.x, v0.screen_pos.y};
    math::Vec2i p1 = {v1.screen_pos.x, v1.screen_pos.y};
    math::Vec2i p2 = {v2.screen_pos.x, v2.screen_pos.y};

    // 2. Compute bounding box of the triangle
    int min_x = std::min({p0.x(), p1.x(), p2.x()});
    int max_x = std::max({p0.x(), p1.x(), p2.x()});
    int min_y = std::min({p0.y(), p1.y(), p2.y()});
    int max_y = std::max({p0.y(), p1.y(), p2.y()});

    // Back-face handling: accept both windings; only reject degenerate
    float total_area = (p1 - p0).cross(p2 - p0);
    if (total_area == 0.0f) {
      return;
    }
    float inv_total_area = 1.0f / total_area;

    // Precompute per-vertex 1/w for perspective-correct interpolation
    float inv_w0_v = 1.0f / v0.varyings.clip_pos.w();
    float inv_w1_v = 1.0f / v1.varyings.clip_pos.w();
    float inv_w2_v = 1.0f / v2.varyings.clip_pos.w();

    for (int y = min_y; y <= max_y; ++y) {
      for (int x = min_x; x <= max_x; ++x) {
        math::Vec2i p = {x, y};

        // 3. Compute barycentric coordinates (signed areas)
        int w0_signed_area = (p1 - p).cross(p2 - p);
        int w1_signed_area = (p2 - p).cross(p0 - p);
        int w2_signed_area = (p0 - p).cross(p1 - p);

        // 4. If pixel is inside the triangle (or on its edges)
        bool inside = (total_area > 0.0f)
                          ? (w0_signed_area >= 0 && w1_signed_area >= 0 &&
                             w2_signed_area >= 0)
                          : (w0_signed_area <= 0 && w1_signed_area <= 0 &&
                             w2_signed_area <= 0);
        if (inside) {
          float alpha = w0_signed_area * inv_total_area;
          float beta = w1_signed_area * inv_total_area;
          float gamma = w2_signed_area * inv_total_area;

          // 5. Perform perspective-correct interpolation
          float inv_w = alpha * inv_w0_v + beta * inv_w1_v + gamma * inv_w2_v;
          float w = 1.0f / inv_w;

          // Interpolate varyings perspective-correctly.
          auto interp_varyings = (v0.varyings * (inv_w0_v * alpha) +
                                  v1.varyings * (inv_w1_v * beta) +
                                  v2.varyings * (inv_w2_v * gamma)) *
                                 w;

          // Interpolate depth for the z-buffer.
          float interp_depth = alpha * v0.screen_pos.depth +
                               beta * v1.screen_pos.depth +
                               gamma * v2.screen_pos.depth;

          // 6. Optional: compute screen-space gradients for UV if present
          if constexpr (requires { interp_varyings.uv; }) {
            // Right neighbor (x+1, y)
            math::Vec2i pr = {x + 1, y};
            int w0r = (p1 - pr).cross(p2 - pr);
            int w1r = (p2 - pr).cross(p0 - pr);
            int w2r = (p0 - pr).cross(p1 - pr);
            float alphar = w0r * inv_total_area;
            float betar = w1r * inv_total_area;
            float gammar = w2r * inv_total_area;
            float inv_wr =
                alphar * inv_w0_v + betar * inv_w1_v + gammar * inv_w2_v;
            float wr = 1.0f / inv_wr;
            auto vary_r = (v0.varyings * (inv_w0_v * alphar) +
                           v1.varyings * (inv_w1_v * betar) +
                           v2.varyings * (inv_w2_v * gammar)) *
                          wr;

            // Down neighbor (x, y+1)
            math::Vec2i pd = {x, y + 1};
            int w0d = (p1 - pd).cross(p2 - pd);
            int w1d = (p2 - pd).cross(p0 - pd);
            int w2d = (p0 - pd).cross(p1 - pd);
            float alphad = w0d * inv_total_area;
            float betad = w1d * inv_total_area;
            float gammad = w2d * inv_total_area;
            float inv_wd =
                alphad * inv_w0_v + betad * inv_w1_v + gammad * inv_w2_v;
            float wd = 1.0f / inv_wd;
            auto vary_d = (v0.varyings * (inv_w0_v * alphad) +
                           v1.varyings * (inv_w1_v * betad) +
                           v2.varyings * (inv_w2_v * gammad)) *
                          wd;

            float dudx = vary_r.uv.x() - interp_varyings.uv.x();
            float dudy = vary_d.uv.x() - interp_varyings.uv.x();
            float dvdx = vary_r.uv.y() - interp_varyings.uv.y();
            float dvdy = vary_d.uv.y() - interp_varyings.uv.y();

            TextureGradGuard grad_guard(dudx, dudy, dvdx, dvdy);
            fragment_processor({{x, y, interp_depth}, interp_varyings});
          } else {
            // No UV: just process the fragment normally
            fragment_processor({{x, y, interp_depth}, interp_varyings});
          }
        }
      }
    }
  }

  void rasterize_line(
      const Fragment<TShader>& v0, const Fragment<TShader>& v1,
      std::function<void(const Fragment<TShader>&)> fragment_processor) {
    int x0 = v0.screen_pos.x, y0 = v0.screen_pos.y;
    int x1 = v1.screen_pos.x, y1 = v1.screen_pos.y;

    const int dx = std::abs(x1 - x0);
    const int sx = x0 < x1 ? 1 : -1;
    const int dy = -std::abs(y1 - y0);
    const int sy = y0 < y1 ? 1 : -1;

    int error = dx + dy;
    int current_x = x0;
    int current_y = y0;

    const float inv_w0 = 1.0f / v0.varyings.clip_pos.w();
    const float inv_w1 = 1.0f / v1.varyings.clip_pos.w();

    const float total_steps = std::max(dx, -dy);
    float current_step = 0.0f;

    while (true) {
      float t = (total_steps == 0) ? 0.0f : current_step / total_steps;

      float interp_depth =
          (1.0f - t) * v0.screen_pos.depth + t * v1.screen_pos.depth;

      float inv_w = (1.0f - t) * inv_w0 + t * inv_w1;
      float w = 1.0f / inv_w;
      auto interp_varyings =
          (v0.varyings * inv_w0 * (1.0f - t) + v1.varyings * inv_w1 * t) * w;

      fragment_processor(
          {{current_x, current_y, interp_depth}, interp_varyings});

      if (current_x == x1 && current_y == y1) {
        break;
      }

      int e2 = 2 * error;
      if (e2 >= dy) {
        if (current_x == x1) break;
        error += dy;
        current_x += sx;
      }
      if (e2 <= dx) {
        if (current_y == y1) break;
        error += dx;
        current_y += sy;
      }
      current_step += 1.0f;
    }
  }
};

}  // namespace renderer
}  // namespace soft_renderer
