#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "soft_renderer/all.hpp"

namespace examples {

using namespace soft_renderer;
using namespace soft_renderer::core;
using namespace soft_renderer::renderer;
using namespace soft_renderer::math;
using namespace soft_renderer::platform;
using namespace soft_renderer::camera;
using namespace soft_renderer::input;

class ExampleRunner {
 public:
  ExampleRunner(const std::string& title, int width = 800, int height = 600)
      : title_(title), width_(width), height_(height) {}

  template <Shader TShader>
  void run(const std::vector<typename TShader::Vertex>& vertices,
           const std::vector<uint32_t>& indices, PrimitiveTopology topology,
           Camera& camera,
           std::unique_ptr<ICameraController> controller = nullptr,
           std::function<void(typename TShader::Uniforms&)> configure_uniforms =
               std::function<void(typename TShader::Uniforms&)>{}) {
    // Setup window and input
    Window window(title_, width_, height_);
    InputManager input_manager;
    input_manager.bind_to_window(window);

    // Setup default controller if none provided
    if (controller) {
      camera.set_controller(std::move(controller));
    } else {
      camera.set_controller(std::make_unique<OrbitController>());
    }

    ViewportTransform viewport(width_, height_);

    // Prepare a default ground plane at y=0 with normal (0,1,0)
    const float g = 10.0f;
    std::vector<FlatShader::Vertex> ground_vertices = {
        {{-g, 0.0f, -g, 1.0f}, {0.25f, 0.25f, 0.25f, 1.0f}},
        {{g, 0.0f, -g, 1.0f}, {0.25f, 0.25f, 0.25f, 1.0f}},
        {{g, 0.0f, g, 1.0f}, {0.25f, 0.25f, 0.25f, 1.0f}},
        {{-g, 0.0f, g, 1.0f}, {0.25f, 0.25f, 0.25f, 1.0f}},
    };
    std::vector<uint32_t> ground_indices = {0, 1, 2, 0, 2, 3};

    // Prepare axis geometry (origin axes with RGB = XYZ)
    const float axis_len = 1.0f;
    const float arrow_len = 0.15f;
    const float arrow_r = 0.05f;

    // Lines for axes
    std::vector<FlatShader::Vertex> axis_vertices_lines = {
        // X axis (red)
        {{0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f, 1.0f}},
        {{axis_len, 0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f, 1.0f}},
        // Y axis (green)
        {{0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 0.0f, 1.0f}},
        {{0.0f, axis_len, 0.0f, 1.0f}, {0.0f, 1.0f, 0.0f, 1.0f}},
        // Z axis (blue)
        {{0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f, 1.0f}},
        {{0.0f, 0.0f, axis_len, 1.0f}, {0.0f, 0.0f, 1.0f, 1.0f}},
    };
    std::vector<uint32_t> axis_indices_lines = {0, 1, 2, 3, 4, 5};

    // Triangle arrowheads (small pyramids) for axes
    std::vector<FlatShader::Vertex> axis_vertices_tri;
    std::vector<uint32_t> axis_indices_tri;
    axis_vertices_tri.reserve(5 * 3);
    axis_indices_tri.reserve(12 * 3);

    auto append_arrow_pyramid = [&](const Vec4f& base_center, const Vec4f& tip,
                                    const Vec4f& b1, const Vec4f& b2,
                                    const Vec4f& b3, const Vec4f& b4,
                                    const Color& color) {
      uint32_t base_index = static_cast<uint32_t>(axis_vertices_tri.size());
      axis_vertices_tri.push_back({b1, color});
      axis_vertices_tri.push_back({b2, color});
      axis_vertices_tri.push_back({b3, color});
      axis_vertices_tri.push_back({b4, color});
      axis_vertices_tri.push_back({tip, color});
      // Faces around the pyramid (tip with base edges)
      // Winding chosen to face outward approximately
      axis_indices_tri.push_back(base_index + 4);
      axis_indices_tri.push_back(base_index + 0);
      axis_indices_tri.push_back(base_index + 2);

      axis_indices_tri.push_back(base_index + 4);
      axis_indices_tri.push_back(base_index + 2);
      axis_indices_tri.push_back(base_index + 1);

      axis_indices_tri.push_back(base_index + 4);
      axis_indices_tri.push_back(base_index + 1);
      axis_indices_tri.push_back(base_index + 3);

      axis_indices_tri.push_back(base_index + 4);
      axis_indices_tri.push_back(base_index + 3);
      axis_indices_tri.push_back(base_index + 0);
    };

    // X axis arrow (red)
    {
      Vec4f base_center = {axis_len, 0.0f, 0.0f, 1.0f};
      Vec4f tip = {axis_len + arrow_len, 0.0f, 0.0f, 1.0f};
      Vec4f b1 = {axis_len, arrow_r, 0.0f, 1.0f};
      Vec4f b2 = {axis_len, -arrow_r, 0.0f, 1.0f};
      Vec4f b3 = {axis_len, 0.0f, arrow_r, 1.0f};
      Vec4f b4 = {axis_len, 0.0f, -arrow_r, 1.0f};
      append_arrow_pyramid(base_center, tip, b1, b2, b3, b4,
                           {1.0f, 0.0f, 0.0f, 1.0f});
    }

    // Y axis arrow (green)
    {
      Vec4f base_center = {0.0f, axis_len, 0.0f, 1.0f};
      Vec4f tip = {0.0f, axis_len + arrow_len, 0.0f, 1.0f};
      Vec4f b1 = {arrow_r, axis_len, 0.0f, 1.0f};
      Vec4f b2 = {-arrow_r, axis_len, 0.0f, 1.0f};
      Vec4f b3 = {0.0f, axis_len, arrow_r, 1.0f};
      Vec4f b4 = {0.0f, axis_len, -arrow_r, 1.0f};
      append_arrow_pyramid(base_center, tip, b1, b2, b3, b4,
                           {0.0f, 1.0f, 0.0f, 1.0f});
    }

    // Z axis arrow (blue)
    {
      Vec4f base_center = {0.0f, 0.0f, axis_len, 1.0f};
      Vec4f tip = {0.0f, 0.0f, axis_len + arrow_len, 1.0f};
      Vec4f b1 = {arrow_r, 0.0f, axis_len, 1.0f};
      Vec4f b2 = {-arrow_r, 0.0f, axis_len, 1.0f};
      Vec4f b3 = {0.0f, arrow_r, axis_len, 1.0f};
      Vec4f b4 = {0.0f, -arrow_r, axis_len, 1.0f};
      append_arrow_pyramid(base_center, tip, b1, b2, b3, b4,
                           {0.0f, 0.0f, 1.0f, 1.0f});
    }

    // Timing for delta_time calculation
    auto last_time = std::chrono::high_resolution_clock::now();

    // Main loop
    window.main_loop([&](core::FrameBuffer& framebuffer) {
      // Calculate delta time
      auto current_time = std::chrono::high_resolution_clock::now();
      auto delta_time =
          std::chrono::duration<double>(current_time - last_time).count();
      last_time = current_time;

      // Handle camera input
      camera.handle_input(delta_time, input_manager.get_current_input());

      // Setup rendering
      TShader shader;
      Renderer<TShader> renderer(framebuffer, shader);
      Mat4f projection = Mat4f(camera.get_projection_matrix(
          static_cast<float>(width_) / height_, 0.01f, 1000.0f));
      Mat4f view = Mat4f(camera.get_view_matrix());
      Mat4f model = Mat4f::identity();
      Mat4f mvp = projection * view * model;

      typename TShader::Uniforms uniforms{};
      // Populate common fields if present
      if constexpr (requires { uniforms.mvp = mvp; }) {
        uniforms.mvp = mvp;
      }
      if (configure_uniforms) {
        configure_uniforms(uniforms);
      }

      // Render
      framebuffer.clear({0, 0, 0, 255});

      // 1) Draw ground first (y=0 plane), using FlatShader with same MVP
      {
        FlatShader ground_shader;
        Renderer<FlatShader> ground_renderer(framebuffer, ground_shader);
        FlatShader::Uniforms ground_uniforms{.mvp = mvp};
        ground_renderer.draw(ground_vertices, ground_indices, ground_uniforms,
                             PrimitiveTopology::Triangles, viewport);
      }

      // 1.5) Draw origin axes: lines + triangle arrowheads
      {
        FlatShader axis_shader;
        Renderer<FlatShader> axis_renderer(framebuffer, axis_shader);
        FlatShader::Uniforms axis_uniforms{.mvp = mvp};
        // Lines
        axis_renderer.draw(axis_vertices_lines, axis_indices_lines,
                           axis_uniforms, PrimitiveTopology::Lines, viewport);
        // Triangles (pyramids)
        axis_renderer.draw(axis_vertices_tri, axis_indices_tri, axis_uniforms,
                           PrimitiveTopology::Triangles, viewport);
      }

      // 2) Draw main content
      renderer.draw(vertices, indices, uniforms, topology, viewport);

      // Update frame at the end
      input_manager.update_frame();
    });
  }

 private:
  std::string title_;
  int width_;
  int height_;
};

}  // namespace examples