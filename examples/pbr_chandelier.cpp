#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "example_base.hpp"
#include "soft_renderer/src/io/gltf_loader.hpp"
#include "soft_renderer/src/io/image_loader.hpp"

using namespace examples;
using namespace soft_renderer;
using namespace soft_renderer::core;
using namespace soft_renderer::renderer;
using namespace soft_renderer::math;

static bool load_image_rgba8_from_jpg(const std::string& path, ImageRGBA8& out);

int main() {
  // 1) 解析 glTF
  io::GltfMeshData mesh;
  std::string gltf_path =
      "/Users/mason/playground/projects/soft-renderer/assets/gltf/"
      "chinese_chandelier_4k.gltf";
  if (!io::load_gltf_simple(gltf_path, mesh)) {
    std::cerr << "Failed to load glTF: " << gltf_path << std::endl;
    return 1;
  }

  // 2) 相机与控制器
  UnitQuatd look_at_origin_rot(Vec3d(1.0, 0.0, 0.0), M_PI * 0.75);
  PerspectiveCamera camera(Vec3d(0.0, 0.6, 1.2), look_at_origin_rot,
                           M_PI / 3.0);

  // 3) 纹理载入（JPEG）
  TextureRGBA8 base_color_tex;
  TextureRGBA8 normal_tex;
  TextureRGBA8 mr_tex;
  TextureRGBA8 ao_tex;
  {
    auto try_load = [&](const std::string& p, TextureRGBA8& tex) {
      if (p.empty()) return;
      ImageRGBA8 img;
      if (!io::load_image_rgba8(p, img)) return;
      tex.set_base_level(img);
      tex.generate_mipmaps(FilterMode::Bilinear);
    };
    try_load(mesh.base_color_path, base_color_tex);
    try_load(mesh.normal_path, normal_tex);
    try_load(mesh.metallic_roughness_path, mr_tex);
    // 用户目录中还提供了单独的 ao/metal/roughness，可按需覆盖
    try_load(
        "/Users/mason/playground/projects/soft-renderer/assets/gltf/textures/"
        "chinese_chandelier_ao_4k.jpg",
        ao_tex);

    // 如果素材出现RGB/BGR错误，尝试对baseColor做R/B通道交换
    if (!base_color_tex.empty()) {
      auto& img0 = base_color_tex.mip(0);
      for (uint32_t y = 0; y < img0.height(); ++y) {
        for (uint32_t x = 0; x < img0.width(); ++x) {
          RGBA8 c = img0(x, y);
          std::swap(c.r, c.b);
          img0(x, y) = c;
        }
      }
      base_color_tex.generate_mipmaps(FilterMode::Bilinear);
    }

    // 若不带合并的MR贴图，尝试由单独金属/粗糙合成（G=rough, B=metal）
    if (mr_tex.empty()) {
      ImageRGBA8 img_metal, img_rough;
      if (io::load_image_rgba8(
              "/Users/mason/playground/projects/soft-renderer/assets/gltf/"
              "textures/chinese_chandelier_metal_4k.jpg",
              img_metal) &&
          io::load_image_rgba8(
              "/Users/mason/playground/projects/soft-renderer/assets/gltf/"
              "textures/chinese_chandelier_rough_4k.jpg",
              img_rough) &&
          img_metal.width() == img_rough.width() &&
          img_metal.height() == img_rough.height()) {
        ImageRGBA8 img_mr(img_metal.width(), img_metal.height());
        for (uint32_t y = 0; y < img_mr.height(); ++y) {
          for (uint32_t x = 0; x < img_mr.width(); ++x) {
            uint8_t metal = img_metal(x, y).r;
            uint8_t rough = img_rough(x, y).r;
            img_mr(x, y) = RGBA8(0, rough, metal, 255);
          }
        }
        mr_tex.set_base_level(img_mr);
        mr_tex.generate_mipmaps(FilterMode::Bilinear);
      }
    }
  }

  // 4) 运行示例
  ExampleRunner runner("PBR - Chinese Chandelier");
  runner.run<PBRShader>(
      mesh.vertices, mesh.indices, PrimitiveTopology::Triangles, camera,
      std::make_unique<OrbitController>(), [&](PBRShader::Uniforms& u) {
        // 变换：该模型导入需要绕Y轴180°
        Mat4f model = create_rotation(Vec3f(1, 0, 0), static_cast<float>(M_PI));
        u.model = model;
        Mat4f projection =
            Mat4f(camera.get_projection_matrix(800.0 / 600.0, 0.01, 100.0));
        Mat4f view = Mat4f(camera.get_view_matrix());
        u.mvp = projection * view * model;
        u.normal_matrix = Mat3f(model.rotation());
        u.camera_pos_ws = Vec3f(0.0f, 0.6f, 1.2f);

        // 简单方向光（从右上前方照向原点）
        u.dir_light.direction_ws = Vec3f(-0.4f, -1.0f, -0.6f).normalized();
        u.dir_light.radiance = {3.0f, 3.0f, 3.0f, 1.0f};

        // 材质
        u.material.base_color_factor = {1, 1, 1, 1};
        u.material.metallic_factor = 0.0f;  // 默认非金属
        u.material.roughness_factor = 1.0f;
        if (!base_color_tex.empty())
          u.material.base_color_tex = &base_color_tex;
        if (!normal_tex.empty()) u.material.normal_tex = &normal_tex;
        if (!mr_tex.empty()) u.material.mr_tex = &mr_tex;
        if (!ao_tex.empty()) u.material.occlusion_tex = &ao_tex;

        u.material.base_color_sampler.uv_origin = UVOrigin::BottomLeft;
        u.material.normal_sampler.uv_origin = UVOrigin::BottomLeft;
        u.material.mr_sampler.uv_origin = UVOrigin::BottomLeft;
        u.material.occlusion_sampler.uv_origin = UVOrigin::BottomLeft;
        u.material.emissive_sampler.uv_origin = UVOrigin::BottomLeft;

        u.material.base_color_sampler.mag_filter = FilterMode::Bilinear;
        u.material.base_color_sampler.min_filter = FilterMode::Bilinear;
        u.material.base_color_sampler.mip_filter = MipFilterMode::Linear;
        u.material.normal_sampler = u.material.base_color_sampler;
        u.material.mr_sampler = u.material.base_color_sampler;
        u.material.occlusion_sampler = u.material.base_color_sampler;
        u.material.emissive_sampler = u.material.base_color_sampler;
      });

  return 0;
}

// 占位：使用stb_image载入jpg为 RGBA8
static bool load_image_rgba8_from_jpg(const std::string& path,
                                      ImageRGBA8& out) {
  return io::load_image_rgba8(path, out);
}
