#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "../renderer/shader.hpp"

#define TINYGLTF_IMPLEMENTATION
#define TINYGLTF_NO_STB_IMAGE
#define TINYGLTF_NO_STB_IMAGE_WRITE
#define TINYGLTF_NO_INCLUDE_STB_IMAGE
#define TINYGLTF_NO_INCLUDE_STB_IMAGE_WRITE
#include <tiny_gltf.h>

namespace soft_renderer {
namespace io {

using namespace soft_renderer::renderer;

struct GltfMeshData {
  std::vector<PBRShader::Vertex> vertices;
  std::vector<uint32_t> indices;

  std::string base_color_path;
  std::string normal_path;
  std::string metallic_roughness_path;
  std::string occlusion_path;
  std::string emissive_path;
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

  std::vector<float> pos, nrm, uv;
  if (!read_accessor_floats(pos_accessor, 3, pos)) return false;
  if (!read_accessor_floats(nrm_accessor, 3, nrm)) return false;
  if (!read_accessor_floats(uv_accessor, 2, uv)) return false;

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
  for (size_t i = 0; i < vcount; ++i) {
    PBRShader::Vertex v{};
    v.position = {pos[i * 3 + 0], pos[i * 3 + 1], pos[i * 3 + 2], 1.0f};
    v.normal = {nrm[i * 3 + 0], nrm[i * 3 + 1], nrm[i * 3 + 2]};
    v.tangent = {1.0f, 0.0f, 0.0f};
    v.uv = {uv[i * 2 + 0], uv[i * 2 + 1]};
    out.vertices[i] = v;
  }

  if (prim.material >= 0 && prim.material < (int)model.materials.size()) {
    const auto& mat = model.materials[prim.material];
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
