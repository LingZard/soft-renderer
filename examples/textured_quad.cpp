#include <vector>

#include "example_base.hpp"

using namespace examples;
using namespace soft_renderer;
using namespace soft_renderer::core;
using namespace soft_renderer::renderer;
using namespace soft_renderer::math;

int main() {
  // Fullscreen-ish quad in NDC z=0 plane
  std::vector<TexturedShader::Vertex> vertices = {
      {{-1.0f, -1.0f, 0.0f, 1.0f}, {0.0f, 0.0f}},
      {{1.0f, -1.0f, 0.0f, 1.0f}, {1.0f, 0.0f}},
      {{1.0f, 1.0f, 0.0f, 1.0f}, {1.0f, 1.0f}},
      {{-1.0f, 1.0f, 0.0f, 1.0f}, {0.0f, 1.0f}},
  };
  std::vector<uint32_t> indices = {0, 1, 2, 0, 2, 3};

  // Camera looking at origin
  UnitQuatd look_at_origin_rot(Vec3d(1.0, 0.0, 0.0), 0.0);
  PerspectiveCamera camera(Vec3d(0, 0, 1.5), look_at_origin_rot, M_PI / 2.0);

  // Load a texture (replace path as needed)
  ImageRGBA8 img = io::read_tga_image("test_image.tga");
  TextureRGBA8 tex;
  if (!img.data()) {
    tex.allocate(2, 2, 1);
    tex.mip(0)(0, 0) = RGBA8(255, 0, 0);
    tex.mip(0)(1, 0) = RGBA8(0, 255, 0);
    tex.mip(0)(0, 1) = RGBA8(0, 0, 255);
    tex.mip(0)(1, 1) = RGBA8(255, 255, 0);
  } else {
    tex.set_base_level(img);
  }
  tex.generate_mipmaps(FilterMode::Bilinear);

  ExampleRunner runner("Textured Quad");
  runner.run<TexturedShader>(vertices, indices, PrimitiveTopology::Triangles,
                             camera, std::make_unique<OrbitController>());

  return 0;
}