#include <cmath>
#include <format>
#include <iostream>
#include <vector>

#include "core/framebuffer.hpp"
#include "core/vertex.hpp"
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

// Generates a point cloud in the shape of a torus.
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

void render_point_cloud_animation() {
  // 1. Setup scene
  const int width = 800;
  const int height = 600;
  FrameBuffer framebuffer(width, height);
  FlatShader shader;
  Renderer<FlatShader> renderer(framebuffer, shader);

  // 2. Create the point cloud geometry
  const int num_points = 10000;
  auto vertices = create_torus_point_cloud(num_points, 1.5f, 0.5f);
  std::vector<uint32_t> indices(num_points);
  std::iota(indices.begin(), indices.end(), 0);

  // 3. Setup view and projection matrices (they don't change per frame)

  UnitQuatd look_at_origin_rot(Vec3d(1.0, 0.0, 0.0), M_PI * 3 / 4);
  PerspectiveCamera camera(Vec3d(0, 3, 3), look_at_origin_rot, M_PI / 2.0);
  Mat4f view = Mat4f(camera.get_view_matrix());
  // Mat4f view = look_at(Vec3f(0, 3, 3), Vec3f(0, 0, 0), Vec3f(0, 1, 0));
  Mat4f projection = Mat4f(camera.get_projection_matrix(
      static_cast<float>(width) / height, 0.1f, 100.0f));

  // 4. Render 24 frames of a 360-degree rotation
  const int num_frames = 24;
  for (int i = 0; i < num_frames; ++i) {
    float angle_rad = static_cast<float>(i) / num_frames * 2.0f * M_PI;

    // a. Update model matrix for the current frame's rotation
    Mat4f model = create_rotation(Vec3f(0.0f, 1.0f, 0.0f), angle_rad);
    Mat4f mvp = projection * view * model;
    FlatShader::Uniforms uniforms{.mvp = mvp};

    // b. Render the scene
    framebuffer.clear({0, 0, 0, 255});
    renderer.draw(vertices, indices, uniforms, PrimitiveTopology::Points);

    // c. Save output file
    std::string filename = std::format("point_cloud_{:02d}.tga", i);
    if (write_tga_image(filename, framebuffer.color_buffer())) {
      std::cout << "Rendered frame " << i << " to " << filename << std::endl;
    } else {
      std::cerr << "Failed to write " << filename << std::endl;
    }
  }
}

int main(int argc, char** argv) {
  render_point_cloud_animation();
  return 0;
}
