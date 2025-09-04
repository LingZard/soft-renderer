#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "example_base.hpp"

using namespace examples;
using namespace soft_renderer;
using namespace soft_renderer::core;
using namespace soft_renderer::renderer;
using namespace soft_renderer::math;

struct ObjMesh {
  std::vector<TexturedShader::Vertex> vertices;
  std::vector<uint32_t> indices;
};

static bool load_obj_with_uv(const std::string& path, ObjMesh& out) {
  std::ifstream in(path);
  if (!in) return false;
  std::vector<Vec3f> positions;
  std::vector<Vec2f> uvs;

  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream iss(line);
    std::string tag;
    iss >> tag;
    if (tag == "v") {
      float x, y, z;
      iss >> x >> y >> z;
      positions.push_back({x, y, z});
    } else if (tag == "vt") {
      float u, v;
      iss >> u >> v;
      uvs.push_back({u, v});
    } else if (tag == "f") {
      // Triangulated faces assumed. Format: v/t[/n]
      // We'll build a temp list then append to out
      std::vector<std::pair<int, int>> verts;  // (vi, ti)
      std::string vert;
      while (iss >> vert) {
        int vi = 0, ti = 0;
        // parse v/t or v/t/n
        size_t s1 = vert.find('/');
        size_t s2 = std::string::npos;
        if (s1 != std::string::npos) s2 = vert.find('/', s1 + 1);
        if (s1 == std::string::npos) {
          vi = std::stoi(vert);
        } else if (s2 == std::string::npos) {
          vi = std::stoi(vert.substr(0, s1));
          ti = std::stoi(vert.substr(s1 + 1));
        } else {
          vi = std::stoi(vert.substr(0, s1));
          std::string tstr = vert.substr(s1 + 1, s2 - s1 - 1);
          ti = tstr.empty() ? 0 : std::stoi(tstr);
        }
        verts.push_back({vi, ti});
      }
      // fan triangulation if polygon
      for (size_t i = 1; i + 1 < verts.size(); ++i) {
        std::pair<int, int> v0 = verts[0];
        std::pair<int, int> v1 = verts[i];
        std::pair<int, int> v2 = verts[i + 1];
        TexturedShader::Vertex tv0{}, tv1{}, tv2{};
        auto fetch = [&](std::pair<int, int> vt, TexturedShader::Vertex& outv) {
          int vi = vt.first;
          int ti = vt.second;
          if (vi < 0) vi = static_cast<int>(positions.size()) + vi + 1;
          if (ti < 0) ti = static_cast<int>(uvs.size()) + ti + 1;
          Vec3f p = positions[static_cast<size_t>(vi - 1)];
          outv.position = {p.x(), p.y(), p.z(), 1.0f};
          if (ti > 0) {
            outv.uv = uvs[static_cast<size_t>(ti - 1)];
          } else {
            outv.uv = {0.0f, 0.0f};
          }
        };
        fetch(v0, tv0);
        fetch(v1, tv1);
        fetch(v2, tv2);
        uint32_t base = static_cast<uint32_t>(out.vertices.size());
        out.vertices.push_back(tv0);
        out.vertices.push_back(tv1);
        out.vertices.push_back(tv2);
        out.indices.push_back(base + 0);
        out.indices.push_back(base + 1);
        out.indices.push_back(base + 2);
      }
    }
  }
  return !out.vertices.empty() && !out.indices.empty();
}

int main() {
  // Load OBJ
  ObjMesh mesh;
  std::string obj_path =
      "/Users/mason/playground/projects/soft-renderer/assets/obj/african_head/"
      "african_head.obj";
  if (!load_obj_with_uv(obj_path, mesh)) {
    printf("Failed to load %s\n", obj_path.c_str());
    return 1;
  }

  // Camera
  UnitQuatd look_at_origin_rot(Vec3d(1.0, 0.0, 0.0), M_PI);
  PerspectiveCamera camera(Vec3d(0, 0, 1.5), look_at_origin_rot,
                           M_PI * 2.0 / 3.0);

  // Load diffuse texture
  ImageRGBA8 img = io::read_tga_image(
      "/Users/mason/playground/projects/soft-renderer/assets/obj/african_head/"
      "african_head_diffuse.tga");
  TextureRGBA8 tex;
  if (!img.data()) {
    printf("Failed to load diffuse texture.\n");
    return 1;
  }
  tex.set_base_level(img);
  tex.generate_mipmaps(FilterMode::Bilinear);

  // Sampler
  SamplerState sampler;
  sampler.address_u = AddressMode::ClampToEdge;
  sampler.address_v = AddressMode::ClampToEdge;
  sampler.min_filter = FilterMode::Bilinear;
  sampler.mag_filter = FilterMode::Bilinear;
  sampler.mip_filter = MipFilterMode::Linear;
  sampler.uv_origin = UVOrigin::TopLeft;

  ExampleRunner runner("Textured OBJ: African Head");
  runner.run<TexturedShader>(
      mesh.vertices, mesh.indices, PrimitiveTopology::Triangles, camera,
      std::make_unique<OrbitController>(), [&](TexturedShader::Uniforms& u) {
        u.texture = &tex;
        u.sampler = sampler;
      });

  return 0;
}