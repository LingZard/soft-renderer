#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "color.hpp"
#include "image.hpp"

namespace soft_renderer {
namespace core {

enum class AddressMode { ClampToEdge, Repeat, MirrorRepeat, ClampToBorder };

enum class FilterMode { Nearest, Bilinear };

enum class MipFilterMode { None, Nearest, Linear };

enum class ColorSpace { Linear, SRGB };

enum class UVOrigin { TopLeft, BottomLeft };

struct SamplerState {
  AddressMode address_u = AddressMode::ClampToEdge;
  AddressMode address_v = AddressMode::ClampToEdge;
  FilterMode min_filter = FilterMode::Bilinear;
  FilterMode mag_filter = FilterMode::Bilinear;
  MipFilterMode mip_filter = MipFilterMode::None;
  UVOrigin uv_origin = UVOrigin::BottomLeft;
  ColorSpace texture_color_space = ColorSpace::Linear;
  RGBA8 border_color = RGBA8(0, 0, 0, 255);
  float lod_bias = 0.0f;
  float min_lod = 0.0f;
  float max_lod = 1000.0f;
  float max_anisotropy = 1.0f;
};

template <typename PixelType>
class Texture2D {
 public:
  using Pixel = PixelType;
  using ImageType = Image<PixelType>;

  Texture2D();
  explicit Texture2D(uint32_t width, uint32_t height, uint32_t mip_levels = 1);
  explicit Texture2D(const ImageType& base_level);
  Texture2D(const Texture2D& other);
  Texture2D(Texture2D&& other) noexcept;
  Texture2D& operator=(const Texture2D& other);
  Texture2D& operator=(Texture2D&& other) noexcept;
  ~Texture2D();

  bool empty() const;
  uint32_t width() const;
  uint32_t height() const;
  uint32_t num_mip_levels() const;
  bool has_mipmaps() const;

  static uint32_t compute_full_mip_count(uint32_t width, uint32_t height);
  bool is_level_valid(uint32_t level) const;
  std::pair<uint32_t, uint32_t> level_dimensions(uint32_t level) const;

  const ImageType& mip(uint32_t level) const;
  ImageType& mip(uint32_t level);
  const PixelType* data(uint32_t level = 0) const;
  PixelType* data(uint32_t level = 0);

  void allocate(uint32_t width, uint32_t height, uint32_t mip_levels);
  void set_base_level(const ImageType& image);
  void set_level(uint32_t level, const ImageType& image);
  void clear(const PixelType& value);
  void clear_mipmaps(uint32_t keep_levels = 1);

  void generate_mipmaps(FilterMode filter = FilterMode::Bilinear);

  PixelType sample_uv(float u, float v, const SamplerState& sampler) const;
  PixelType sample_lod(float u, float v, float lod,
                       const SamplerState& sampler) const;
  PixelType sample_grad(float u, float v, float dudx, float dudy, float dvdx,
                        float dvdy, const SamplerState& sampler) const;
  PixelType sample_texel(int x, int y, uint32_t level = 0) const;

 private:
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  std::vector<ImageType> mip_levels_;
};

using TextureRGBA8 = Texture2D<RGBA8>;

}  // namespace core
}  // namespace soft_renderer