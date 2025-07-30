#include <cmath>
#include <iostream>
#include <vector>

#include "example_base.hpp"

using namespace examples;
using namespace soft_renderer;
using namespace soft_renderer::core;
using namespace soft_renderer::renderer;
using namespace soft_renderer::math;

// Generates vertices and line indices for a sphere.
std::pair<std::vector<FlatShader::Vertex>, std::vector<uint32_t>> create_sphere(
    int segments, int rings) {
  std::vector<FlatShader::Vertex> vertices;
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

int main() {
  auto [vertices, indices] = create_sphere(32, 16);

  UnitQuatd look_at_origin_rot(Vec3d(1.0, 0.0, 0.0), M_PI * 3.0 / 4.0);
  PerspectiveCamera camera(Vec3d(0, 0.4, 0.4), look_at_origin_rot, M_PI / 2.0);

  ExampleRunner runner("Wireframe Sphere Example");
  runner.run<FlatShader>(vertices, indices, PrimitiveTopology::Lines, camera);

  return 0;
}