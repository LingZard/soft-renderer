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
           std::unique_ptr<ICameraController> controller = nullptr) {
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
          static_cast<float>(width_) / height_, 0.1f, 100.0f));
      Mat4f view = Mat4f(camera.get_view_matrix());
      Mat4f model = Mat4f::identity();
      Mat4f mvp = projection * view * model;

      typename TShader::Uniforms uniforms{};
      // Populate common fields if present
      if constexpr (requires { uniforms.mvp = mvp; }) {
        uniforms.mvp = mvp;
      }

      // Render
      framebuffer.clear({0, 0, 0, 255});
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