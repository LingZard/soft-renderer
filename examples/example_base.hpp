#pragma once

#include <functional>
#include <string>

#include "soft_renderer/all.hpp"

namespace examples {

using namespace soft_renderer;
using namespace soft_renderer::core;
using namespace soft_renderer::renderer;
using namespace soft_renderer::math;
using namespace soft_renderer::platform;

class ExampleRunner {
 public:
  ExampleRunner(const std::string& title, int width = 800, int height = 600)
      : title_(title), width_(width), height_(height) {}

  template <Shader TShader>
  void run(const std::vector<typename TShader::Vertex>& vertices,
           const std::vector<uint32_t>& indices, PrimitiveTopology topology,
           PerspectiveCamera& camera);

 private:
  void handle_mouse_button(int button, int mod, bool is_down) {
    if (button == platform::MouseButton::LEFT) {
      mouse_down_ = is_down;
      if (is_down && window_ptr_) {
        int x, y;
        window_ptr_->get_mouse_position(x, y);
        last_mouse_x_ = static_cast<double>(x);
        last_mouse_y_ = static_cast<double>(y);
      }
    }
  }
  void handle_mouse_move(int x, int y) {
    if (mouse_down_ && camera_ptr_) {
      double current_x = static_cast<double>(x);
      double current_y = static_cast<double>(y);

      double dx = current_x - last_mouse_x_;
      double dy = current_y - last_mouse_y_;

      last_mouse_x_ = current_x;
      last_mouse_y_ = current_y;

      if (dx != 0 || dy != 0) {
        // Rotation sensitivity
        const double sensitivity = 0.0025;
        // Invert dx for natural orbital controls (drag left moves object left)
        UnitQuatd rot_x(Vec3d(0.0, 1.0, 0.0), -dx * sensitivity);
        UnitQuatd rot_y(Vec3d(1.0, 0.0, 0.0), -dy * sensitivity);
        camera_ptr_->rotate(rot_x * rot_y);
      }
    }
  }

  std::string title_;
  int width_, height_;

  // Mouse interaction state
  bool mouse_down_ = false;
  double last_mouse_x_ = 0.0;
  double last_mouse_y_ = 0.0;
  PerspectiveCamera* camera_ptr_ = nullptr;
  Window* window_ptr_ = nullptr;
};

template <Shader TShader>
void ExampleRunner::run(const std::vector<typename TShader::Vertex>& vertices,
                        const std::vector<uint32_t>& indices,
                        PrimitiveTopology topology, PerspectiveCamera& camera) {
  // Setup window
  Window window(title_, width_, height_);
  window_ptr_ = &window;

  // Setup mouse callbacks
  window.set_mouse_button_callback([this](int button, int mod, bool is_down) {
    handle_mouse_button(button, mod, is_down);
  });
  window.set_mouse_move_callback(
      [this](int x, int y) { handle_mouse_move(x, y); });

  // Setup scene
  FlatShader shader;
  ViewportTransform viewport(width_, height_);
  camera_ptr_ = &camera;

  // Setup matrices
  Mat4f projection = Mat4f(camera.get_projection_matrix(
      static_cast<float>(width_) / height_, 0.1f, 100.0f));
  Mat4f model = Mat4f::identity();

  // Main loop
  window.main_loop([&](core::FrameBuffer& framebuffer) {
    Renderer<FlatShader> renderer(framebuffer, shader);

    Mat4f view = Mat4f(camera.get_view_matrix());
    Mat4f mvp = projection * view * model;
    FlatShader::Uniforms uniforms{.mvp = mvp};

    framebuffer.clear({20, 20, 20, 255});
    renderer.draw(vertices, indices, uniforms, topology, viewport);
  });

  camera_ptr_ = nullptr;
  window_ptr_ = nullptr;
}

}  // namespace examples