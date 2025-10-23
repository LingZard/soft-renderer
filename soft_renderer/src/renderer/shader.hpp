#pragma once

#include <concepts>

#include "../core/color.hpp"
#include "../core/math/mat.hpp"
#include "../core/math/vec.hpp"
#include "../core/texture.hpp"

namespace soft_renderer {
namespace renderer {

using namespace soft_renderer::core;

class FlatShader {
 public:
  struct Vertex {
    Vec4f position;
    Color color;
  };

  struct Uniforms {
    Mat4f mvp;
  };

  struct Varyings {
    Vec4f clip_pos;
    Color color;

    Varyings operator+(const Varyings& other) const {
      return {.clip_pos = clip_pos + other.clip_pos,
              .color = color + other.color};
    }

    Varyings operator-(const Varyings& other) const {
      return {.clip_pos = clip_pos - other.clip_pos,
              .color = color - other.color};
    }

    Varyings operator*(float scalar) const {
      return {.clip_pos = clip_pos * scalar, .color = color * scalar};
    }
  };

  Varyings vertex(const Vertex& v, const Uniforms& u) const {
    return {
        .clip_pos = u.mvp * v.position,
        .color = v.color,
    };
  };

  Color fragment(const Varyings& vary, const Uniforms& u) const {
    return vary.color;
  };
};

class TexturedShader {
 public:
  struct Vertex {
    Vec4f position;
    Vec2f uv;
  };

  struct Uniforms {
    Mat4f mvp;
    const TextureRGBA8* texture = nullptr;
    SamplerState sampler;
  };

  struct Varyings {
    Vec4f clip_pos;
    Vec2f uv;

    Varyings operator+(const Varyings& other) const {
      return {.clip_pos = clip_pos + other.clip_pos, .uv = uv + other.uv};
    }

    Varyings operator-(const Varyings& other) const {
      return {.clip_pos = clip_pos - other.clip_pos, .uv = uv - other.uv};
    }

    Varyings operator*(float scalar) const {
      return {.clip_pos = clip_pos * scalar, .uv = uv * scalar};
    }
  };

  Varyings vertex(const Vertex& v, const Uniforms& u) const {
    return {
        .clip_pos = u.mvp * v.position,
        .uv = v.uv,
    };
  };

  Color fragment(const Varyings& vary, const Uniforms& u) const {
    if (u.texture == nullptr || u.texture->empty()) {
      return {1.0f, 0.0f, 1.0f, 1.0f};
    }
    RGBA8 c = u.texture->sample(vary.uv.x(), vary.uv.y(), u.sampler);
    return rgba8_to_linear_color(c);
  };
};

class PBRShader {
 public:
  struct Vertex {
    Vec4f position;
    Vec3f normal;
    Vec3f tangent;
    float tangent_sign = 1.0f;  // +1 or -1, from glTF TANGENT.w
    Vec2f uv;
    Vec2f uv1;  // optional second UV set
  };

  struct DirectionalLight {
    Vec3f direction_ws;  // Incident light direction at the surface (from point
                         // to light)
    Color radiance;
  };

  struct Material {
    // Factors
    Color base_color_factor{1.0f, 1.0f, 1.0f, 1.0f};
    float metallic_factor = 1.0f;
    float roughness_factor = 1.0f;
    float normal_scale = 1.0f;        // glTF normalTexture.scale
    float occlusion_strength = 1.0f;  // glTF occlusionTexture.strength
    Color emissive_factor{0.0f, 0.0f, 0.0f, 1.0f};  // glTF emissiveFactor (RGB)
    float emissive_strength = 1.0f;  // KHR_materials_emissive_strength

    // Specular/IOR (KHR_materials_specular, KHR_materials_ior)
    float ior = 1.5f;              // index of refraction for dielectric F0
    float specular_factor = 1.0f;  // scales dielectric F0 intensity
    Color specular_color_factor{1.0f, 1.0f, 1.0f, 1.0f};

    // Alpha and facing policies
    enum class AlphaMode { Opaque, Mask, Blend };
    AlphaMode alpha_mode = AlphaMode::Opaque;
    float alpha_cutoff = 0.5f;
    bool double_sided = false;

    // Textures (optional)
    const TextureRGBA8* base_color_tex = nullptr;  // SRGB by default
    const TextureRGBA8* mr_tex =
        nullptr;  // G=roughness, B=metallic (glTF style)
    const TextureRGBA8* normal_tex = nullptr;          // tangent-space normal
    const TextureRGBA8* occlusion_tex = nullptr;       // R=AO
    const TextureRGBA8* emissive_tex = nullptr;        // SRGB by default
    const TextureRGBA8* specular_tex = nullptr;        // intensity (linear)
    const TextureRGBA8* specular_color_tex = nullptr;  // color (linear)
    // Clearcoat removed

    // Samplers per-texture (allow different color spaces)
    SamplerState base_color_sampler{.texture_color_space = ColorSpace::SRGB};
    SamplerState mr_sampler{.texture_color_space = ColorSpace::Linear};
    SamplerState normal_sampler{.texture_color_space = ColorSpace::Linear};
    SamplerState occlusion_sampler{.texture_color_space = ColorSpace::Linear};
    SamplerState emissive_sampler{.texture_color_space = ColorSpace::SRGB};
    SamplerState specular_sampler{.texture_color_space = ColorSpace::Linear};
    SamplerState specular_color_sampler{.texture_color_space =
                                            ColorSpace::Linear};
    // Clearcoat samplers removed

    // UV set selection and KHR_texture_transform
    struct TexTransform {
      Vec2f offset{0.0f, 0.0f};
      Vec2f scale{1.0f, 1.0f};
      float rotation = 0.0f;  // radians
      Vec2f center{0.0f, 0.0f};
    };
    int base_color_texcoord = 0;
    int mr_texcoord = 0;
    int normal_texcoord = 0;
    int occlusion_texcoord = 1;  // glTF default often 1
    int emissive_texcoord = 0;
    int specular_texcoord = 0;
    int specular_color_texcoord = 0;
    // Clearcoat texcoords removed
    TexTransform base_color_xform{};
    TexTransform mr_xform{};
    TexTransform normal_xform{};
    TexTransform occlusion_xform{};
    TexTransform emissive_xform{};
    TexTransform specular_xform{};
    TexTransform specular_color_xform{};
    // Clearcoat transforms removed
  };

  struct Uniforms {
    Mat4f mvp = Mat4f::identity();
    Mat4f model = Mat4f::identity();
    Mat3f normal_matrix =
        Mat3f::identity();  // usually inverse-transpose of model 3x3
    Vec3f camera_pos_ws{0.0f, 0.0f, 0.0f};
    DirectionalLight dir_light{};
    Material material{};
  };

  struct Varyings {
    Vec4f clip_pos;
    Vec3f pos_ws;
    Vec3f normal_ws;
    Vec3f tangent_ws;
    float tangent_sign;
    Vec2f uv;
    Vec2f uv1;

    Varyings operator+(const Varyings& other) const {
      return {.clip_pos = clip_pos + other.clip_pos,
              .pos_ws = pos_ws + other.pos_ws,
              .normal_ws = normal_ws + other.normal_ws,
              .tangent_ws = tangent_ws + other.tangent_ws,
              .tangent_sign = tangent_sign + other.tangent_sign,
              .uv = uv + other.uv,
              .uv1 = uv1 + other.uv1};
    }

    Varyings operator-(const Varyings& other) const {
      return {.clip_pos = clip_pos - other.clip_pos,
              .pos_ws = pos_ws - other.pos_ws,
              .normal_ws = normal_ws - other.normal_ws,
              .tangent_ws = tangent_ws - other.tangent_ws,
              .tangent_sign = tangent_sign - other.tangent_sign,
              .uv = uv - other.uv,
              .uv1 = uv1 - other.uv1};
    }

    Varyings operator*(float scalar) const {
      return {.clip_pos = clip_pos * scalar,
              .pos_ws = pos_ws * scalar,
              .normal_ws = normal_ws * scalar,
              .tangent_ws = tangent_ws * scalar,
              .tangent_sign = tangent_sign * scalar,
              .uv = uv * scalar,
              .uv1 = uv1 * scalar};
    }
  };

  Varyings vertex(const Vertex& v, const Uniforms& u) const {
    Vec4f pos_ws4 = u.model * v.position;
    Vec3f pos_ws = pos_ws4.xyz();
    Vec3f n_ws = (u.normal_matrix * v.normal).normalized();
    // Orthonormalize T against N to improve stability with arbitrary inputs
    Vec3f t_ws = (u.normal_matrix * v.tangent);
    t_ws = (t_ws - n_ws * n_ws.dot(t_ws)).normalized();

    return {
        .clip_pos = u.mvp * v.position,
        .pos_ws = pos_ws,
        .normal_ws = n_ws,
        .tangent_ws = t_ws,
        .tangent_sign = v.tangent_sign,
        .uv = v.uv,
        .uv1 = v.uv1,
    };
  };

  Color fragment(const Varyings& vary, const Uniforms& u) const {
    // Helpers
    auto apply_tex_transform = [&](const Vec2f& in_uv,
                                   const Material::TexTransform& xf) {
      float s = std::sin(xf.rotation);
      float c = std::cos(xf.rotation);
      Vec2f d = in_uv - xf.center;
      Vec2f rot{c * d.x() - s * d.y(), s * d.x() + c * d.y()};
      Vec2f r = rot + xf.center;
      return Vec2f{r.x() * xf.scale.x() + xf.offset.x(),
                   r.y() * xf.scale.y() + xf.offset.y()};
    };
    auto choose_uv = [&](int texcoord_index) -> Vec2f {
      return (texcoord_index == 1) ? vary.uv1 : vary.uv;
    };
    auto uv_for = [&](int texcoord_index, const Material::TexTransform& xf) {
      Vec2f base_uv = choose_uv(texcoord_index);
      return apply_tex_transform(base_uv, xf);
    };
    auto srgb_tex_to_linear = [&](const TextureRGBA8* tex, float uvs, float vvs,
                                  const SamplerState& s) -> Color {
      if (tex == nullptr || tex->empty()) return {1, 1, 1, 1};
      RGBA8 c = tex->sample(uvs, vvs, s);
      if (s.texture_color_space == ColorSpace::SRGB) {
        return rgba8_to_linear_color(c);
      } else {
        // Linear: simple 0..255 -> 0..1 mapping without gamma
        return {
            static_cast<float>(c.r) / 255.0f, static_cast<float>(c.g) / 255.0f,
            static_cast<float>(c.b) / 255.0f, static_cast<float>(c.a) / 255.0f};
      }
    };

    auto sample_channel = [&](const TextureRGBA8* tex, float uvs, float vvs,
                              const SamplerState& s, int ch) -> float {
      if (tex == nullptr || tex->empty()) return 1.0f;
      RGBA8 c = tex->sample(uvs, vvs, s);
      uint8_t v = 255;
      switch (ch) {
        case 0:
          v = c.r;
          break;
        case 1:
          v = c.g;
          break;
        case 2:
          v = c.b;
          break;
        case 3:
          v = c.a;
          break;
      }
      if (s.texture_color_space == ColorSpace::SRGB) {
        // Decode SRGB per channel as luminance component (approx via color
        // helper) Use linear conversion on that channel only
        float f = static_cast<float>(v) / 255.0f;
        return srgb_to_linear_component(f);
      }
      return static_cast<float>(v) / 255.0f;
    };

    // Fetch material inputs
    const auto& m = u.material;

    // Base Color (RGBA): factor × texture
    Color base_color = m.base_color_factor;
    if (m.base_color_tex && !m.base_color_tex->empty()) {
      Vec2f uvbc = uv_for(m.base_color_texcoord, m.base_color_xform);
      Color tex_c = srgb_tex_to_linear(m.base_color_tex, uvbc.x(), uvbc.y(),
                                       m.base_color_sampler);
      base_color = base_color * tex_c;
    }

    // Metallic-Roughness from factors and texture (glTF ORM: G=Roughness,
    // B=Metallic)
    float metallic = std::clamp(m.metallic_factor, 0.0f, 1.0f);
    float roughness = std::clamp(m.roughness_factor, 0.0f, 1.0f);
    if (m.mr_tex && !m.mr_tex->empty()) {
      Vec2f uvmr = uv_for(m.mr_texcoord, m.mr_xform);
      float tex_rough =
          sample_channel(m.mr_tex, uvmr.x(), uvmr.y(), m.mr_sampler, 1);
      float tex_metal =
          sample_channel(m.mr_tex, uvmr.x(), uvmr.y(), m.mr_sampler, 2);
      roughness = std::clamp(roughness * tex_rough, 0.0f, 1.0f);
      metallic = std::clamp(metallic * tex_metal, 0.0f, 1.0f);
    }

    // Normal
    Vec3f N = vary.normal_ws.normalized();
    if (m.normal_tex && !m.normal_tex->empty()) {
      Vec2f uvn = uv_for(m.normal_texcoord, m.normal_xform);
      RGBA8 nc = m.normal_tex->sample(uvn.x(), uvn.y(), m.normal_sampler);
      Vec3f n_ts =
          Vec3f(nc.r / 255.0f * 2.0f - 1.0f, nc.g / 255.0f * 2.0f - 1.0f,
                nc.b / 255.0f * 2.0f - 1.0f);
      // Apply normal scale to XY per glTF spec
      n_ts =
          Vec3f(n_ts.x() * m.normal_scale, n_ts.y() * m.normal_scale, n_ts.z());
      n_ts = n_ts.normalized();
      Vec3f T = vary.tangent_ws;
      if (T.norm() < 1e-6f) {
        // Fallback: build an orthonormal basis from N
        Vec3f up = std::abs(N.y()) < 0.999f ? Vec3f(0.0f, 1.0f, 0.0f)
                                            : Vec3f(1.0f, 0.0f, 0.0f);
        T = up.cross(N).normalized();
      } else {
        // Orthonormalize to reduce artifacts with skewed tangents
        T = (T - N * N.dot(T)).normalized();
      }
      Vec3f B = (N.cross(T)).normalized() * vary.tangent_sign;
      // TBN with columns [T, B, N]
      Mat3f TBN{T, B, N};
      N = (TBN * n_ts).normalized();
    }

    // View and light
    Vec3f V = (u.camera_pos_ws - vary.pos_ws).normalized();
    if (m.double_sided && N.dot(V) < 0.0f) {
      N = -N;
    }
    Vec3f L = u.dir_light.direction_ws.normalized();
    Vec3f H = (V + L).normalized();

    float NoV = std::max(0.0f, N.dot(V));
    float NoL = std::max(0.0f, N.dot(L));
    float NoH = std::max(0.0f, N.dot(H));
    float VoH = std::max(0.0f, V.dot(H));
    if (NoL <= 0.0f || NoV <= 0.0f) {
      // Unlit: only emissive and alpha are relevant
      Color out = m.emissive_factor;
      if (m.emissive_tex && !m.emissive_tex->empty()) {
        Vec2f uve = uv_for(m.emissive_texcoord, m.emissive_xform);
        Color emissive_tex = srgb_tex_to_linear(m.emissive_tex, uve.x(),
                                                uve.y(), m.emissive_sampler);
        out = Color{out.x() * emissive_tex.x(), out.y() * emissive_tex.y(),
                    out.z() * emissive_tex.z(), 1.0f};
      }
      out = out * m.emissive_strength;
      out.w() = base_color.w();
      return out;
    }

    // Specular inputs (include textures before computing F0/F/specular)
    float ior = std::max(1.0f, m.ior);
    float spec_intensity = m.specular_factor;
    Color spec_color = m.specular_color_factor;
    if (m.specular_tex && !m.specular_tex->empty()) {
      Vec2f uvs = uv_for(m.specular_texcoord, m.specular_xform);
      float spec_t = sample_channel(m.specular_tex, uvs.x(), uvs.y(),
                                    m.specular_sampler, 0);
      spec_intensity *= spec_t;
    }
    if (m.specular_color_tex && !m.specular_color_tex->empty()) {
      Vec2f uvs = uv_for(m.specular_color_texcoord, m.specular_color_xform);
      RGBA8 sc = m.specular_color_tex->sample(uvs.x(), uvs.y(),
                                              m.specular_color_sampler);
      spec_color = Color{static_cast<float>(sc.r) / 255.0f,
                         static_cast<float>(sc.g) / 255.0f,
                         static_cast<float>(sc.b) / 255.0f, 1.0f};
    }

    // Cook–Torrance GGX terms
    float alpha = std::max(roughness * roughness, 1e-3f);
    float alpha2 = alpha * alpha;
    float denom = (NoH * NoH) * (alpha2 - 1.0f) + 1.0f;
    float D = alpha2 / (M_PI * denom * denom + 1e-7f);

    auto G_Smith = [&](float NoX) {
      float k = (alpha + 1.0f);
      k = (k * k) / 8.0f;  // Schlick-GGX for direct lighting
      return NoX / (NoX * (1.0f - k) + k + 1e-7f);
    };
    float G = G_Smith(NoL) * G_Smith(NoV);

    // Fresnel Schlick with specular extensions and metallic mix
    float f0_dielectric = std::pow((ior - 1.0f) / (ior + 1.0f), 2.0f);
    Color F0 = {f0_dielectric * spec_intensity * spec_color.x(),
                f0_dielectric * spec_intensity * spec_color.y(),
                f0_dielectric * spec_intensity * spec_color.z(), 1.0f};
    F0 = F0 * (1.0f - metallic) +
         Color{base_color.x(), base_color.y(), base_color.z(), 1.0f} * metallic;

    auto fresnel_schlick = [&](float cosTheta, const Color& F0c) -> Color {
      float fc = std::pow(std::max(0.0f, 1.0f - cosTheta), 5.0f);
      return F0c + (Color{1.0f, 1.0f, 1.0f, 1.0f} - F0c) * fc;
    };
    Color F = fresnel_schlick(VoH, F0);

    // Specular and diffuse
    Color specular = F * (D * G / std::max(1e-7f, 4.0f * NoV * NoL));
    Color kd = (Color{1.0f, 1.0f, 1.0f, 1.0f} - F) * (1.0f - metallic);
    Color diffuse = kd * (base_color * (1.0f / static_cast<float>(M_PI)));

    // AO affects (approx) only diffuse term in absence of IBL
    float ao = 1.0f;
    if (m.occlusion_tex && !m.occlusion_tex->empty()) {
      Vec2f uvo = uv_for(m.occlusion_texcoord, m.occlusion_xform);
      ao = sample_channel(m.occlusion_tex, uvo.x(), uvo.y(),
                          m.occlusion_sampler, 0);
    }
    Color diffuse_ao =
        diffuse * (ao * m.occlusion_strength + (1.0f - m.occlusion_strength));

    // Lighting
    Color Lo = (diffuse_ao + specular) * u.dir_light.radiance * NoL;

    // (specular extensions already applied above)

    // Emissive
    // Emissive: emissiveFactor × emissiveTexture × emissiveStrength
    Color emissive_term = m.emissive_factor;
    if (m.emissive_tex && !m.emissive_tex->empty()) {
      Vec2f uve = uv_for(m.emissive_texcoord, m.emissive_xform);
      Color emissive_tex = srgb_tex_to_linear(m.emissive_tex, uve.x(), uve.y(),
                                              m.emissive_sampler);
      emissive_term = Color{emissive_term.x() * emissive_tex.x(),
                            emissive_term.y() * emissive_tex.y(),
                            emissive_term.z() * emissive_tex.z(), 1.0f};
    }
    emissive_term = emissive_term * m.emissive_strength;
    Lo += emissive_term;

    // Alpha and alpha mode
    Lo.w() = base_color.w();
    if (m.alpha_mode == Material::AlphaMode::Mask && Lo.w() < m.alpha_cutoff) {
      // Negative alpha indicates discard for pipeline
      Lo.w() = -1.0f;
    }
    return Lo;
  };
};

}  // namespace renderer
}  // namespace soft_renderer