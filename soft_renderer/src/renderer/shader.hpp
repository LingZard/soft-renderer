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
    Vec2f uv;
  };

  struct DirectionalLight {
    Vec3f direction_ws;
    Color radiance;
  };

  struct Material {
    // Factors
    Color base_color_factor{1.0f, 1.0f, 1.0f, 1.0f};
    float metallic_factor = 1.0f;
    float roughness_factor = 1.0f;

    // Textures (optional)
    const TextureRGBA8* base_color_tex = nullptr;  // SRGB by default
    const TextureRGBA8* mr_tex =
        nullptr;  // G=roughness, B=metallic (glTF style)
    const TextureRGBA8* normal_tex = nullptr;     // tangent-space normal
    const TextureRGBA8* occlusion_tex = nullptr;  // R=AO
    const TextureRGBA8* emissive_tex = nullptr;   // SRGB by default

    // Samplers per-texture (allow different color spaces)
    SamplerState base_color_sampler{.texture_color_space = ColorSpace::SRGB};
    SamplerState mr_sampler{.texture_color_space = ColorSpace::Linear};
    SamplerState normal_sampler{.texture_color_space = ColorSpace::Linear};
    SamplerState occlusion_sampler{.texture_color_space = ColorSpace::Linear};
    SamplerState emissive_sampler{.texture_color_space = ColorSpace::SRGB};
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
    Vec2f uv;

    Varyings operator+(const Varyings& other) const {
      return {.clip_pos = clip_pos + other.clip_pos,
              .pos_ws = pos_ws + other.pos_ws,
              .normal_ws = normal_ws + other.normal_ws,
              .tangent_ws = tangent_ws + other.tangent_ws,
              .uv = uv + other.uv};
    }

    Varyings operator-(const Varyings& other) const {
      return {.clip_pos = clip_pos - other.clip_pos,
              .pos_ws = pos_ws - other.pos_ws,
              .normal_ws = normal_ws - other.normal_ws,
              .tangent_ws = tangent_ws - other.tangent_ws,
              .uv = uv - other.uv};
    }

    Varyings operator*(float scalar) const {
      return {.clip_pos = clip_pos * scalar,
              .pos_ws = pos_ws * scalar,
              .normal_ws = normal_ws * scalar,
              .tangent_ws = tangent_ws * scalar,
              .uv = uv * scalar};
    }
  };

  Varyings vertex(const Vertex& v, const Uniforms& u) const {
    Vec4f pos_ws4 = u.model * v.position;
    Vec3f pos_ws = pos_ws4.xyz();
    Vec3f n_ws = (u.normal_matrix * v.normal).normalized();
    Vec3f t_ws = (u.normal_matrix * v.tangent).normalized();

    return {
        .clip_pos = u.mvp * v.position,
        .pos_ws = pos_ws,
        .normal_ws = n_ws,
        .tangent_ws = t_ws,
        .uv = v.uv,
    };
  };

  Color fragment(const Varyings& vary, const Uniforms& u) const {
    // Helpers
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

    // Base Color
    Color base_color = m.base_color_factor;
    if (m.base_color_tex && !m.base_color_tex->empty()) {
      base_color =
          base_color * srgb_tex_to_linear(m.base_color_tex, vary.uv.x(),
                                          vary.uv.y(), m.base_color_sampler);
    }
    base_color.w() = m.base_color_factor.w();

    // Metallic-Roughness from factors and texture (glTF ORM: G=Roughness,
    // B=Metallic)
    float metallic = m.metallic_factor;
    float roughness = m.roughness_factor;
    if (m.mr_tex && !m.mr_tex->empty()) {
      float tex_rough =
          sample_channel(m.mr_tex, vary.uv.x(), vary.uv.y(), m.mr_sampler, 1);
      float tex_metal =
          sample_channel(m.mr_tex, vary.uv.x(), vary.uv.y(), m.mr_sampler, 2);
      roughness = std::clamp(roughness * tex_rough, 0.04f, 1.0f);
      metallic = std::clamp(metallic * tex_metal, 0.0f, 1.0f);
    } else {
      roughness = std::clamp(roughness, 0.04f, 1.0f);
      metallic = std::clamp(metallic, 0.0f, 1.0f);
    }

    // Normal
    Vec3f N = vary.normal_ws.normalized();
    if (m.normal_tex && !m.normal_tex->empty()) {
      RGBA8 nc =
          m.normal_tex->sample(vary.uv.x(), vary.uv.y(), m.normal_sampler);
      Vec3f n_ts =
          Vec3f(nc.r / 255.0f * 2.0f - 1.0f, nc.g / 255.0f * 2.0f - 1.0f,
                nc.b / 255.0f * 2.0f - 1.0f)
              .normalized();
      Vec3f T = vary.tangent_ws;
      if (T.norm() < 1e-6f) {
        // Fallback: build an orthonormal basis from N
        Vec3f up = std::abs(N.y()) < 0.999f ? Vec3f(0.0f, 1.0f, 0.0f)
                                            : Vec3f(1.0f, 0.0f, 0.0f);
        T = up.cross(N).normalized();
      } else {
        T = T.normalized();
      }
      Vec3f B = N.cross(T).normalized();
      // TBN with columns [T B N]
      Mat3f TBN{
          Vec3f{T.x(), B.x(), N.x()},
          Vec3f{T.y(), B.y(), N.y()},
          Vec3f{T.z(), B.z(), N.z()},
      };
      N = (TBN * n_ts).normalized();
    }

    // View and light
    Vec3f V = (u.camera_pos_ws - vary.pos_ws).normalized();
    Vec3f L = u.dir_light.direction_ws.normalized();
    Vec3f H = (V + L).normalized();

    float NoV = std::max(0.0f, N.dot(V));
    float NoL = std::max(0.0f, N.dot(L));
    float NoH = std::max(0.0f, N.dot(H));
    float VoH = std::max(0.0f, V.dot(H));

    // Cook–Torrance GGX
    float alpha = roughness * roughness;
    float alpha2 = alpha * alpha;
    float denom = (NoH * NoH) * (alpha2 - 1.0f) + 1.0f;
    float D = alpha2 / (M_PI * denom * denom + 1e-7f);

    auto G_Smith = [&](float NoX) {
      float k = (alpha + 1.0f);
      k = (k * k) / 8.0f;  // Schlick-GGX for direct lighting
      return NoX / (NoX * (1.0f - k) + k + 1e-7f);
    };
    float G = G_Smith(NoL) * G_Smith(NoV);

    // Fresnel Schlick
    Color F0 = {0.04f, 0.04f, 0.04f, 1.0f};
    F0 = F0 * (1.0f - metallic) +
         Color{base_color.x(), base_color.y(), base_color.z(), 1.0f} * metallic;
    auto fresnel_schlick = [&](float cosTheta, const Color& F0c) -> Color {
      float fc = std::pow(std::max(0.0f, 1.0f - cosTheta), 5.0f);
      return F0c + (Color{1.0f, 1.0f, 1.0f, 1.0f} - F0c) * fc;
    };
    Color F = fresnel_schlick(VoH, F0);

    // Specular
    Color specular = F * (D * G / std::max(1e-7f, 4.0f * NoV * NoL));

    // Diffuse (Lambert)
    Color kd = (Color{1.0f, 1.0f, 1.0f, 1.0f} - F) * (1.0f - metallic);
    Color diffuse = kd * (base_color * (1.0f / static_cast<float>(M_PI)));

    // Lighting
    Color Lo = (diffuse + specular) * u.dir_light.radiance * NoL;

    // AO
    float ao = 1.0f;
    if (m.occlusion_tex && !m.occlusion_tex->empty()) {
      ao = sample_channel(m.occlusion_tex, vary.uv.x(), vary.uv.y(),
                          m.occlusion_sampler, 0);
    }
    Lo = Lo * ao;

    // Emissive
    if (m.emissive_tex && !m.emissive_tex->empty()) {
      Color emissive = srgb_tex_to_linear(m.emissive_tex, vary.uv.x(),
                                          vary.uv.y(), m.emissive_sampler);
      Lo += emissive;
    }

    // Alpha from base color factor
    Lo.w() = base_color.w();
    return Lo;
  };
};

}  // namespace renderer
}  // namespace soft_renderer