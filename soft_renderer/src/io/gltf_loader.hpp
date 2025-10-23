#pragma once

#include <tiny_gltf.h>

#include <cstdint>
#include <string>
#include <vector>

#include "../renderer/shader.hpp"

namespace soft_renderer {
namespace io {

using namespace soft_renderer::renderer;
using namespace soft_renderer::core;
using namespace soft_renderer::math;

struct GltfMeshData {
  std::vector<PBRShader::Vertex> vertices;
  std::vector<uint32_t> indices;

  std::string base_color_path;
  std::string normal_path;
  std::string metallic_roughness_path;
  std::string occlusion_path;
  std::string emissive_path;
  std::string specular_path;
  std::string specular_color_path;
  // Clearcoat removed

  // Material factors (glTF PBR Metallic-Roughness)
  Color base_color_factor{1.0f, 1.0f, 1.0f, 1.0f};
  float metallic_factor = 1.0f;
  float roughness_factor = 1.0f;
  float normal_scale = 1.0f;        // normalTexture.scale
  float occlusion_strength = 1.0f;  // occlusionTexture.strength
  Color emissive_factor{0.0f, 0.0f, 0.0f, 1.0f};
  float emissive_strength = 1.0f;  // KHR_materials_emissive_strength
  float ior = 1.5f;                // KHR_materials_ior
  float specular_factor = 1.0f;    // KHR_materials_specular.specularFactor
  Color specular_color_factor{1.0f, 1.0f, 1.0f, 1.0f};
  // Clearcoat removed

  // Pipeline-related hints
  std::string alpha_mode = "OPAQUE";  // OPAQUE/MASK/BLEND
  float alpha_cutoff = 0.5f;          // for MASK
  bool double_sided = false;

  // UV set usage and KHR_texture_transform per-texture
  // texCoord indices
  int base_color_texcoord = 0;
  int mr_texcoord = 0;
  int normal_texcoord = 0;
  int occlusion_texcoord = 1;  // often 1 for AO
  int emissive_texcoord = 0;
  int specular_texcoord = 0;
  int specular_color_texcoord = 0;
  // Clearcoat texcoords removed

  struct TexTransform {
    Vec2f offset{0.0f, 0.0f};
    Vec2f scale{1.0f, 1.0f};
    float rotation = 0.0f;
    Vec2f center{0.0f, 0.0f};
  };
  TexTransform base_color_xform{};
  TexTransform mr_xform{};
  TexTransform normal_xform{};
  TexTransform occlusion_xform{};
  TexTransform emissive_xform{};
  TexTransform specular_xform{};
  TexTransform specular_color_xform{};
  // Clearcoat transforms removed
};

static inline std::string _gltf_parent_dir(const std::string& path) {
  size_t pos = path.find_last_of("/\\");
  if (pos == std::string::npos) return std::string(".");
  return path.substr(0, pos);
}

static inline bool load_gltf_simple(const std::string& gltf_file,
                                    GltfMeshData& out) {
  tinygltf::Model model;
  tinygltf::TinyGLTF loader;
  std::string err, warn;
  bool ok = false;
  loader.SetImageLoader(
      [](tinygltf::Image*, int, std::string*, std::string*, int, int,
         const unsigned char*, int, void*) { return true; },
      nullptr);
  if (gltf_file.size() >= 5 &&
      gltf_file.substr(gltf_file.size() - 5) == ".glb") {
    ok = loader.LoadBinaryFromFile(&model, &err, &warn, gltf_file);
  } else {
    ok = loader.LoadASCIIFromFile(&model, &err, &warn, gltf_file);
  }
  if (!ok) return false;
  if (model.meshes.empty()) return false;
  const auto& mesh = model.meshes[0];
  if (mesh.primitives.empty()) return false;
  const auto& prim = mesh.primitives[0];

  int pos_accessor =
      prim.attributes.count("POSITION") ? prim.attributes.at("POSITION") : -1;
  int nrm_accessor =
      prim.attributes.count("NORMAL") ? prim.attributes.at("NORMAL") : -1;
  int uv_accessor = prim.attributes.count("TEXCOORD_0")
                        ? prim.attributes.at("TEXCOORD_0")
                        : -1;
  int tan_accessor =
      prim.attributes.count("TANGENT") ? prim.attributes.at("TANGENT") : -1;
  int uv1_accessor = prim.attributes.count("TEXCOORD_1")
                         ? prim.attributes.at("TEXCOORD_1")
                         : -1;
  if (pos_accessor < 0 || nrm_accessor < 0 || uv_accessor < 0 ||
      prim.indices < 0)
    return false;

  auto read_accessor_floats = [&](int accessor_index, int num_comp,
                                  std::vector<float>& out_data) -> bool {
    const auto& acc = model.accessors[accessor_index];
    const auto& bv = model.bufferViews[acc.bufferView];
    const auto& buf = model.buffers[bv.buffer];
    size_t stride = acc.ByteStride(bv);
    if (stride == 0) stride = num_comp * sizeof(float);
    out_data.resize(acc.count * num_comp);
    const uint8_t* base = buf.data.data() + bv.byteOffset + acc.byteOffset;
    for (size_t i = 0; i < acc.count; ++i) {
      const float* src = reinterpret_cast<const float*>(base + i * stride);
      for (int c = 0; c < num_comp; ++c) out_data[i * num_comp + c] = src[c];
    }
    return true;
  };

  std::vector<float> pos, nrm, uv, uv1, tan;
  if (!read_accessor_floats(pos_accessor, 3, pos)) return false;
  if (!read_accessor_floats(nrm_accessor, 3, nrm)) return false;
  if (!read_accessor_floats(uv_accessor, 2, uv)) return false;
  bool has_tangent = false;
  if (tan_accessor >= 0) {
    if (!read_accessor_floats(tan_accessor, 4, tan)) return false;
    has_tangent = true;
  }
  bool has_uv1 = false;
  if (uv1_accessor >= 0) {
    if (!read_accessor_floats(uv1_accessor, 2, uv1)) return false;
    has_uv1 = true;
  }

  const auto& acc_idx = model.accessors[prim.indices];
  const auto& bv_idx = model.bufferViews[acc_idx.bufferView];
  const auto& buf_idx = model.buffers[bv_idx.buffer];
  const uint8_t* base_idx =
      buf_idx.data.data() + bv_idx.byteOffset + acc_idx.byteOffset;
  out.indices.resize(acc_idx.count);
  if (acc_idx.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) {
    const uint16_t* p = reinterpret_cast<const uint16_t*>(base_idx);
    for (size_t i = 0; i < acc_idx.count; ++i) out.indices[i] = p[i];
  } else if (acc_idx.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT) {
    const uint32_t* p = reinterpret_cast<const uint32_t*>(base_idx);
    for (size_t i = 0; i < acc_idx.count; ++i) out.indices[i] = p[i];
  } else if (acc_idx.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(base_idx);
    for (size_t i = 0; i < acc_idx.count; ++i) out.indices[i] = p[i];
  } else {
    return false;
  }

  size_t vcount = model.accessors[pos_accessor].count;
  out.vertices.resize(vcount);

  // If glTF doesn't provide tangents, generate them from positions and UVs.
  std::vector<Vec3f> gen_tangent(vcount, Vec3f(0.0f, 0.0f, 0.0f));
  std::vector<Vec3f> gen_bitangent(vcount, Vec3f(0.0f, 0.0f, 0.0f));
  if (!has_tangent) {
    for (size_t f = 0; f + 2 < out.indices.size(); f += 3) {
      uint32_t i0 = out.indices[f + 0];
      uint32_t i1 = out.indices[f + 1];
      uint32_t i2 = out.indices[f + 2];
      // Positions
      Vec3f p0{pos[i0 * 3 + 0], pos[i0 * 3 + 1], pos[i0 * 3 + 2]};
      Vec3f p1{pos[i1 * 3 + 0], pos[i1 * 3 + 1], pos[i1 * 3 + 2]};
      Vec3f p2{pos[i2 * 3 + 0], pos[i2 * 3 + 1], pos[i2 * 3 + 2]};
      // UVs
      Vec2f uv0{uv[i0 * 2 + 0], uv[i0 * 2 + 1]};
      Vec2f uv1{uv[i1 * 2 + 0], uv[i1 * 2 + 1]};
      Vec2f uv2{uv[i2 * 2 + 0], uv[i2 * 2 + 1]};

      Vec3f dp1 = p1 - p0;
      Vec3f dp2 = p2 - p0;
      float du1 = uv1.x() - uv0.x();
      float dv1 = uv1.y() - uv0.y();
      float du2 = uv2.x() - uv0.x();
      float dv2 = uv2.y() - uv0.y();
      float denom = du1 * dv2 - dv1 * du2;
      if (std::abs(denom) < 1e-8f) {
        continue;  // degenerate UVs; skip
      }
      float r = 1.0f / denom;
      Vec3f t = (dp1 * dv2 - dp2 * dv1) * r;
      Vec3f b = (dp2 * du1 - dp1 * du2) * r;

      gen_tangent[i0] = gen_tangent[i0] + t;
      gen_tangent[i1] = gen_tangent[i1] + t;
      gen_tangent[i2] = gen_tangent[i2] + t;
      gen_bitangent[i0] = gen_bitangent[i0] + b;
      gen_bitangent[i1] = gen_bitangent[i1] + b;
      gen_bitangent[i2] = gen_bitangent[i2] + b;
    }
  }

  for (size_t i = 0; i < vcount; ++i) {
    PBRShader::Vertex v{};
    v.position = {pos[i * 3 + 0], pos[i * 3 + 1], pos[i * 3 + 2], 1.0f};
    v.normal = {nrm[i * 3 + 0], nrm[i * 3 + 1], nrm[i * 3 + 2]};
    if (has_tangent) {
      v.tangent = {tan[i * 4 + 0], tan[i * 4 + 1], tan[i * 4 + 2]};
      v.tangent_sign = tan[i * 4 + 3];
    } else {
      Vec3f N{v.normal.x(), v.normal.y(), v.normal.z()};
      Vec3f T = gen_tangent[i];
      Vec3f B = gen_bitangent[i];
      if (T.norm() < 1e-8f) {
        // Fallback: build an orthonormal basis from N
        Vec3f up = std::abs(N.y()) < 0.999f ? Vec3f(0.0f, 1.0f, 0.0f)
                                            : Vec3f(1.0f, 0.0f, 0.0f);
        T = up.cross(N).normalized();
        B = N.cross(T);
      }
      // Orthonormalize T and compute handedness sign from accumulated B
      T = (T - N * N.dot(T)).normalized();
      Vec3f B_recon = N.cross(T);
      float sign = (B_recon.dot(B) < 0.0f) ? -1.0f : 1.0f;
      v.tangent = {T.x(), T.y(), T.z()};
      v.tangent_sign = sign;
    }
    v.uv = {uv[i * 2 + 0], uv[i * 2 + 1]};
    v.uv1 = has_uv1 ? Vec2f{uv1[i * 2 + 0], uv1[i * 2 + 1]}
                    : Vec2f{v.uv.x(), v.uv.y()};
    out.vertices[i] = v;
  }

  if (prim.material >= 0 && prim.material < (int)model.materials.size()) {
    const auto& mat = model.materials[prim.material];
    // Textures (compatible with older tinygltf interfaces)
    if (mat.values.count("baseColorTexture")) {
      int texIndex = mat.values.at("baseColorTexture").TextureIndex();
      if (texIndex >= 0 && texIndex < (int)model.textures.size()) {
        int imgIndex = model.textures[texIndex].source;
        if (imgIndex >= 0 && imgIndex < (int)model.images.size())
          out.base_color_path = model.images[imgIndex].uri;
      }
    }
    if (mat.additionalValues.count("normalTexture")) {
      int texIndex = mat.additionalValues.at("normalTexture").TextureIndex();
      if (texIndex >= 0 && texIndex < (int)model.textures.size()) {
        int imgIndex = model.textures[texIndex].source;
        if (imgIndex >= 0 && imgIndex < (int)model.images.size())
          out.normal_path = model.images[imgIndex].uri;
      }
    }
    if (mat.values.count("metallicRoughnessTexture")) {
      int texIndex = mat.values.at("metallicRoughnessTexture").TextureIndex();
      if (texIndex >= 0 && texIndex < (int)model.textures.size()) {
        int imgIndex = model.textures[texIndex].source;
        if (imgIndex >= 0 && imgIndex < (int)model.images.size())
          out.metallic_roughness_path = model.images[imgIndex].uri;
      }
    }
    if (mat.additionalValues.count("occlusionTexture")) {
      int texIndex = mat.additionalValues.at("occlusionTexture").TextureIndex();
      if (texIndex >= 0 && texIndex < (int)model.textures.size()) {
        int imgIndex = model.textures[texIndex].source;
        if (imgIndex >= 0 && imgIndex < (int)model.images.size())
          out.occlusion_path = model.images[imgIndex].uri;
      }
    }
    if (mat.additionalValues.count("emissiveTexture")) {
      int texIndex = mat.additionalValues.at("emissiveTexture").TextureIndex();
      if (texIndex >= 0 && texIndex < (int)model.textures.size()) {
        int imgIndex = model.textures[texIndex].source;
        if (imgIndex >= 0 && imgIndex < (int)model.images.size())
          out.emissive_path = model.images[imgIndex].uri;
      }
    }
    // Optional KHR_materials_specular textures
    auto itSpec = mat.extensions.find("KHR_materials_specular");
    if (itSpec != mat.extensions.end()) {
      const tinygltf::Value& ext = itSpec->second;
      if (ext.Has("specularTexture")) {
        const auto& st = ext.Get("specularTexture");
        if (st.Has("index")) {
          int texIndex = st.Get("index").GetNumberAsInt();
          if (texIndex >= 0 && texIndex < (int)model.textures.size()) {
            int imgIndex = model.textures[texIndex].source;
            if (imgIndex >= 0 && imgIndex < (int)model.images.size())
              out.specular_path = model.images[imgIndex].uri;
          }
        }
      }
      if (ext.Has("specularColorTexture")) {
        const auto& sct = ext.Get("specularColorTexture");
        if (sct.Has("index")) {
          int texIndex = sct.Get("index").GetNumberAsInt();
          if (texIndex >= 0 && texIndex < (int)model.textures.size()) {
            int imgIndex = model.textures[texIndex].source;
            if (imgIndex >= 0 && imgIndex < (int)model.images.size())
              out.specular_color_path = model.images[imgIndex].uri;
          }
        }
      }
    }
    // KHR_materials_clearcoat not supported: ignored

    // Factors via modern tinygltf fields
    if (!mat.pbrMetallicRoughness.baseColorFactor.empty() &&
        mat.pbrMetallicRoughness.baseColorFactor.size() >= 4) {
      out.base_color_factor = Color{
          static_cast<float>(mat.pbrMetallicRoughness.baseColorFactor[0]),
          static_cast<float>(mat.pbrMetallicRoughness.baseColorFactor[1]),
          static_cast<float>(mat.pbrMetallicRoughness.baseColorFactor[2]),
          static_cast<float>(mat.pbrMetallicRoughness.baseColorFactor[3])};
    }
    if (mat.pbrMetallicRoughness.metallicFactor >= 0.0) {
      out.metallic_factor =
          static_cast<float>(mat.pbrMetallicRoughness.metallicFactor);
    }
    if (mat.pbrMetallicRoughness.roughnessFactor >= 0.0) {
      out.roughness_factor =
          static_cast<float>(mat.pbrMetallicRoughness.roughnessFactor);
    }
    // normal scale
    if (mat.normalTexture.index >= 0) {
      out.normal_scale = static_cast<float>(mat.normalTexture.scale);
    }
    // occlusion strength
    if (mat.occlusionTexture.index >= 0) {
      out.occlusion_strength =
          static_cast<float>(mat.occlusionTexture.strength);
    }
    // emissive factor
    if (!mat.emissiveFactor.empty() && mat.emissiveFactor.size() >= 3) {
      out.emissive_factor =
          Color{static_cast<float>(mat.emissiveFactor[0]),
                static_cast<float>(mat.emissiveFactor[1]),
                static_cast<float>(mat.emissiveFactor[2]), 1.0f};
    }
    // emissive strength extension
    auto itExt = mat.extensions.find("KHR_materials_emissive_strength");
    if (itExt != mat.extensions.end()) {
      const tinygltf::Value& ext = itExt->second;
      if (ext.Has("emissiveStrength") &&
          ext.Get("emissiveStrength").IsNumber()) {
        out.emissive_strength =
            static_cast<float>(ext.Get("emissiveStrength").Get<double>());
      }
    }
    // IOR
    auto itIor = mat.extensions.find("KHR_materials_ior");
    if (itIor != mat.extensions.end()) {
      const tinygltf::Value& ext = itIor->second;
      if (ext.Has("ior") && ext.Get("ior").IsNumber()) {
        out.ior = static_cast<float>(ext.Get("ior").Get<double>());
      }
    }
    // Specular
    auto itSp = mat.extensions.find("KHR_materials_specular");
    if (itSp != mat.extensions.end()) {
      const tinygltf::Value& ext = itSp->second;
      if (ext.Has("specularFactor") && ext.Get("specularFactor").IsNumber()) {
        out.specular_factor =
            static_cast<float>(ext.Get("specularFactor").Get<double>());
      }
      if (ext.Has("specularColorFactor") &&
          ext.Get("specularColorFactor").IsArray() &&
          ext.Get("specularColorFactor").ArrayLen() >= 3) {
        out.specular_color_factor =
            Color{static_cast<float>(
                      ext.Get("specularColorFactor").Get(0).Get<double>()),
                  static_cast<float>(
                      ext.Get("specularColorFactor").Get(1).Get<double>()),
                  static_cast<float>(
                      ext.Get("specularColorFactor").Get(2).Get<double>()),
                  1.0f};
      }
    }
    // pipeline hints
    out.alpha_mode = mat.alphaMode;  // default "OPAQUE"
    out.alpha_cutoff = static_cast<float>(mat.alphaCutoff);
    out.double_sided = mat.doubleSided;

    // texCoord indices
    if (mat.pbrMetallicRoughness.baseColorTexture.index >= 0)
      out.base_color_texcoord =
          mat.pbrMetallicRoughness.baseColorTexture.texCoord;
    if (mat.pbrMetallicRoughness.metallicRoughnessTexture.index >= 0)
      out.mr_texcoord =
          mat.pbrMetallicRoughness.metallicRoughnessTexture.texCoord;
    if (mat.normalTexture.index >= 0)
      out.normal_texcoord = mat.normalTexture.texCoord;
    if (mat.occlusionTexture.index >= 0)
      out.occlusion_texcoord = mat.occlusionTexture.texCoord;
    if (mat.emissiveTexture.index >= 0)
      out.emissive_texcoord = mat.emissiveTexture.texCoord;
    // Clearcoat texcoord ignored
    if (itSpec != mat.extensions.end()) {
      const tinygltf::Value& ext = itSpec->second;
      if (ext.Has("specularTexture") &&
          ext.Get("specularTexture").Has("texCoord"))
        out.specular_texcoord =
            ext.Get("specularTexture").Get("texCoord").GetNumberAsInt();
      if (ext.Has("specularColorTexture") &&
          ext.Get("specularColorTexture").Has("texCoord"))
        out.specular_color_texcoord =
            ext.Get("specularColorTexture").Get("texCoord").GetNumberAsInt();
    }

    auto read_transform = [&](const tinygltf::Value& ext,
                              GltfMeshData::TexTransform& dst) {
      if (ext.Has("offset") && ext.Get("offset").IsArray() &&
          ext.Get("offset").ArrayLen() >= 2) {
        dst.offset =
            Vec2f{static_cast<float>(ext.Get("offset").Get(0).Get<double>()),
                  static_cast<float>(ext.Get("offset").Get(1).Get<double>())};
      }
      if (ext.Has("scale") && ext.Get("scale").IsArray() &&
          ext.Get("scale").ArrayLen() >= 2) {
        dst.scale =
            Vec2f{static_cast<float>(ext.Get("scale").Get(0).Get<double>()),
                  static_cast<float>(ext.Get("scale").Get(1).Get<double>())};
      }
      if (ext.Has("rotation") && ext.Get("rotation").IsNumber()) {
        dst.rotation = static_cast<float>(ext.Get("rotation").Get<double>());
      }
      if (ext.Has("center") && ext.Get("center").IsArray() &&
          ext.Get("center").ArrayLen() >= 2) {
        dst.center =
            Vec2f{static_cast<float>(ext.Get("center").Get(0).Get<double>()),
                  static_cast<float>(ext.Get("center").Get(1).Get<double>())};
      }
    };

    auto itBC = mat.pbrMetallicRoughness.baseColorTexture.extensions.find(
        "KHR_texture_transform");
    if (itBC != mat.pbrMetallicRoughness.baseColorTexture.extensions.end())
      read_transform(itBC->second, out.base_color_xform);
    auto itMR =
        mat.pbrMetallicRoughness.metallicRoughnessTexture.extensions.find(
            "KHR_texture_transform");
    if (itMR !=
        mat.pbrMetallicRoughness.metallicRoughnessTexture.extensions.end())
      read_transform(itMR->second, out.mr_xform);
    auto itN = mat.normalTexture.extensions.find("KHR_texture_transform");
    if (itN != mat.normalTexture.extensions.end())
      read_transform(itN->second, out.normal_xform);
    auto itO = mat.occlusionTexture.extensions.find("KHR_texture_transform");
    if (itO != mat.occlusionTexture.extensions.end())
      read_transform(itO->second, out.occlusion_xform);
    auto itE = mat.emissiveTexture.extensions.find("KHR_texture_transform");
    if (itE != mat.emissiveTexture.extensions.end())
      read_transform(itE->second, out.emissive_xform);
    // Clearcoat transforms ignored
    if (itSpec != mat.extensions.end()) {
      const tinygltf::Value& ext = itSpec->second;
      if (ext.Has("specularTexture") &&
          ext.Get("specularTexture").Has("extensions")) {
        const auto& ex = ext.Get("specularTexture").Get("extensions");
        if (ex.Has("KHR_texture_transform"))
          read_transform(ex.Get("KHR_texture_transform"), out.specular_xform);
      }
      if (ext.Has("specularColorTexture") &&
          ext.Get("specularColorTexture").Has("extensions")) {
        const auto& ex = ext.Get("specularColorTexture").Get("extensions");
        if (ex.Has("KHR_texture_transform"))
          read_transform(ex.Get("KHR_texture_transform"),
                         out.specular_color_xform);
      }
    }
  }

  auto fix_path = [&](std::string& p) {
    if (p.empty()) return;
    if (p.rfind("/", 0) == 0 || p.find(":/") != std::string::npos) return;
    p = _gltf_parent_dir(gltf_file) + "/" + p;
  };
  fix_path(out.base_color_path);
  fix_path(out.normal_path);
  fix_path(out.metallic_roughness_path);
  fix_path(out.occlusion_path);
  fix_path(out.emissive_path);
  return true;
}

}  // namespace io
}  // namespace soft_renderer
