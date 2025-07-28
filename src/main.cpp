#include <MiniFB.h>

#include <cmath>
#include <format>
#include <iostream>
#include <numeric>
#include <vector>

#include "core/framebuffer.hpp"
#include "core/primitive.hpp"
#include "graphics/camera.hpp"
#include "graphics/renderer.hpp"
#include "graphics/shader.hpp"
#include "io/tga_image.hpp"
#include "math/mat.hpp"
#include "math/vec.hpp"

using namespace soft_renderer;
using namespace soft_renderer::core;
using namespace soft_renderer::graphics;
using namespace soft_renderer::io;
using namespace soft_renderer::math;

// --- Global state for mouse input ---
static bool g_mouse_down = false;
static double g_last_mouse_x = 0.0;
static double g_last_mouse_y = 0.0;
static PerspectiveCamera* g_camera_ptr = nullptr;

// --- minifb callbacks ---
void mouse_button_callback(struct mfb_window* window, mfb_mouse_button button,
                           mfb_key_mod mod, bool is_down) {
  if (button == MOUSE_BTN_1) {
    g_mouse_down = is_down;
    g_last_mouse_x = mfb_get_mouse_x(window);
    g_last_mouse_y = mfb_get_mouse_y(window);
  }
}

void mouse_move_callback(struct mfb_window* window, int x, int y) {
  if (g_mouse_down && g_camera_ptr) {
    double current_x = static_cast<double>(x);
    double current_y = static_cast<double>(y);

    double dx = current_x - g_last_mouse_x;
    double dy = current_y - g_last_mouse_y;

    g_last_mouse_x = current_x;
    g_last_mouse_y = current_y;

    if (dx != 0 || dy != 0) {
      // Rotation sensitivity
      const double sensitivity = 0.0025;
      // Invert dx for natural orbital controls (drag left moves object left)
      UnitQuatd rot_x(Vec3d(0.0, 1.0, 0.0), -dx * sensitivity);
      UnitQuatd rot_y(Vec3d(1.0, 0.0, 0.0), -dy * sensitivity);
      g_camera_ptr->rotate(rot_x * rot_y);
    }
  }
}

template <Shader TShader>
void run_interactive_viewer(
    const std::vector<typename TShader::Vertex>& vertices,
    const std::vector<uint32_t>& indices, PrimitiveTopology topology,
    PerspectiveCamera& camera) {
  const int width = 800;
  const int height = 600;

  // 1. Setup window
  struct mfb_window* window =
      mfb_open_ex("Soft Renderer", width, height, WF_RESIZABLE);
  if (!window) {
    return;
  }
  mfb_set_mouse_button_callback(window, mouse_button_callback);
  mfb_set_mouse_move_callback(window, mouse_move_callback);

  // 2. Setup scene
  FrameBuffer framebuffer(width, height);
  FlatShader shader;
  Renderer<FlatShader> renderer(framebuffer, shader);
  ViewportTransform viewport(width, height);
  g_camera_ptr = &camera;

  // 3. Setup projection and model matrices (they don't change)
  Mat4f projection = Mat4f(camera.get_projection_matrix(
      static_cast<float>(width) / height, 0.1f, 100.0f));
  Mat4f model = Mat4f::identity();

  // 4. Main loop
  do {
    // a. Update view matrix from camera (controlled by mouse)
    Mat4f view = Mat4f(camera.get_view_matrix());
    Mat4f mvp = projection * view * model;
    FlatShader::Uniforms uniforms{.mvp = mvp};

    // b. Render the scene
    framebuffer.clear({20, 20, 20, 255});
    renderer.draw(vertices, indices, uniforms, topology, viewport);

    // c. Update window
    int state = mfb_update_ex(window,
                              reinterpret_cast<void*>(const_cast<core::RGBA8*>(
                                  framebuffer.color_buffer().data())),
                              width, height);
    if (state < 0) {
      break;
    }
  } while (mfb_wait_sync(window));

  mfb_close(window);
  g_camera_ptr = nullptr;
}

namespace flat_shader {
using Vertex = FlatShader::Vertex;

// Generates vertices for a torus point cloud.
std::vector<Vertex> create_torus_point_cloud(int num_points, float R, float r) {
  std::vector<Vertex> vertices;
  vertices.reserve(num_points);

  for (int i = 0; i < num_points; ++i) {
    float u = static_cast<float>(rand()) / RAND_MAX * 2.0f * M_PI;
    float v = static_cast<float>(rand()) / RAND_MAX * 2.0f * M_PI;

    float x = (R + r * cos(v)) * cos(u);
    float y = r * sin(v);
    float z = (R + r * cos(v)) * sin(u);

    // Color based on the parametric coordinates to create a nice pattern
    float red = 0.5f * (1.0f + cos(u));
    float green = 0.5f * (1.0f + sin(v));
    float blue = 0.5f * (1.0f + sin(u));

    vertices.push_back(
        {.position = {x, y, z, 1.0f}, .color = {red, green, blue, 1.0f}});
  }
  return vertices;
}

// Generates vertices and line indices for a sphere.
std::pair<std::vector<Vertex>, std::vector<uint32_t>> create_sphere(
    int segments, int rings) {
  std::vector<Vertex> vertices;
  std::vector<uint32_t> indices;

  // Generate vertices
  for (int i = 0; i <= rings; ++i) {
    float v = static_cast<float>(i) / rings;
    float phi = v * M_PI;
    for (int j = 0; j <= segments; ++j) {
      float u = static_cast<float>(j) / segments;
      float theta = u * 2.0f * M_PI;

      float x = cos(theta) * sin(phi);
      float y = cos(phi);
      float z = sin(theta) * sin(phi);

      vertices.push_back(
          {.position = {x, y, z, 1.0f}, .color = {u, v, 1.0f - u, 1.0f}});
    }
  }

  // Generate indices for lines
  for (int i = 0; i < rings; ++i) {
    for (int j = 0; j < segments; ++j) {
      uint32_t current = i * (segments + 1) + j;
      uint32_t next_j = i * (segments + 1) + (j + 1);
      uint32_t next_i = (i + 1) * (segments + 1) + j;
      uint32_t next_ij = (i + 1) * (segments + 1) + (j + 1);

      // Ring lines
      indices.push_back(current);
      indices.push_back(next_j);

      // Segment lines
      indices.push_back(current);
      indices.push_back(next_i);
    }
  }

  return {vertices, indices};
}

// Generates vertices and triangle indices for a sphere.
std::pair<std::vector<Vertex>, std::vector<uint32_t>> create_sphere_triangles(
    int segments, int rings) {
  std::vector<Vertex> vertices;
  std::vector<uint32_t> indices;

  // Generate vertices
  for (int i = 0; i <= rings; ++i) {
    float v = static_cast<float>(i) / rings;
    float phi = v * M_PI;
    for (int j = 0; j <= segments; ++j) {
      float u = static_cast<float>(j) / segments;
      float theta = u * 2.0f * M_PI;

      float x = cos(theta) * sin(phi);
      float y = cos(phi);
      float z = sin(theta) * sin(phi);

      vertices.push_back(
          {.position = {x, y, z, 1.0f}, .color = {u, v, 1.0f - u, 1.0f}});
    }
  }

  // Generate indices for triangles
  for (int i = 0; i < rings; ++i) {
    for (int j = 0; j < segments; ++j) {
      uint32_t current = i * (segments + 1) + j;
      uint32_t next_j = i * (segments + 1) + (j + 1);
      uint32_t next_i = (i + 1) * (segments + 1) + j;
      uint32_t next_ij = (i + 1) * (segments + 1) + (j + 1);

      // First triangle
      indices.push_back(current);
      indices.push_back(next_i);
      indices.push_back(next_j);

      // Second triangle
      indices.push_back(next_j);
      indices.push_back(next_i);
      indices.push_back(next_ij);
    }
  }

  return {vertices, indices};
}

void show_point_cloud() {
  const int num_points = 10000;
  auto vertices = create_torus_point_cloud(num_points, 1.5f, 0.5f);
  std::vector<uint32_t> indices(num_points);
  std::iota(indices.begin(), indices.end(), 0);

  UnitQuatd look_at_origin_rot(Vec3d(1.0, 0.0, 0.0), M_PI * 3.0 / 4.0);
  PerspectiveCamera camera(Vec3d(0, 3, 3), look_at_origin_rot, M_PI / 2.0);

  run_interactive_viewer<FlatShader>(vertices, indices,
                                     PrimitiveTopology::Points, camera);
}

void show_wireframe_sphere() {
  auto [vertices, indices] = create_sphere(32, 16);

  UnitQuatd look_at_origin_rot(Vec3d(1.0, 0.0, 0.0), M_PI * 3.0 / 4.0);
  PerspectiveCamera camera(Vec3d(0, 0.4, 0.4), look_at_origin_rot, M_PI / 2.0);

  run_interactive_viewer<FlatShader>(vertices, indices,
                                     PrimitiveTopology::Lines, camera);
}

void show_triangle_sphere() {
  auto [vertices, indices] = create_sphere_triangles(32, 16);

  UnitQuatd look_at_origin_rot(Vec3d(1.0, 0.0, 0.0), M_PI * 3.0 / 4.0);
  PerspectiveCamera camera(Vec3d(0, 1, 2), look_at_origin_rot, M_PI / 2.0);

  run_interactive_viewer<FlatShader>(vertices, indices,
                                     PrimitiveTopology::Triangles, camera);
}
}  // namespace flat_shader

int main(int argc, char** argv) {
  flat_shader::show_triangle_sphere();
  // show_wireframe_sphere();
  // show_point_cloud();
  return 0;
}
