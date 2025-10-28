#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

#include "example_base.hpp"

using namespace examples;
using namespace soft_renderer;
using namespace soft_renderer::core;
using namespace soft_renderer::renderer;
using namespace soft_renderer::math;

// Generates vertices for a torus point cloud.
std::vector<FlatShader::Vertex> create_torus_point_cloud(int num_points,
                                                         float R, float r) {
  std::vector<FlatShader::Vertex> vertices;
  vertices.reserve(num_points);

  for (int i = 0; i < num_points; ++i) {
    float u = static_cast<float>(rand()) / RAND_MAX * 2.0f *
              static_cast<float>(std::numbers::pi);
    float v = static_cast<float>(rand()) / RAND_MAX * 2.0f *
              static_cast<float>(std::numbers::pi);

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

int main() {
  const int num_points = 10000;
  auto vertices = create_torus_point_cloud(num_points, 1.5f, 0.5f);
  std::vector<uint32_t> indices(num_points);
  std::iota(indices.begin(), indices.end(), 0);

  UnitQuatd look_at_origin_rot(Vec3d(1.0, 0.0, 0.0),
                               std::numbers::pi * 3.0 / 4.0);
  PerspectiveCamera camera(Vec3d(0, 3, 3), look_at_origin_rot,
                           std::numbers::pi / 2.0);

  ExampleRunner runner("Point Cloud Example - Press 1-4 to switch controllers");
  runner.run<FlatShader>(vertices, indices, PrimitiveTopology::Points, camera);

  return 0;
}