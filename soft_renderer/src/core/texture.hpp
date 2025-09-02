#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <type_traits>
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
  void set_mip_chain(const std::vector<ImageType>& levels);
  void clear(const PixelType& value);

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

  // Helpers
  static int wrap_repeat(int i, int dim);
  static int wrap_mirror(int i, int dim);
  static int clamp_index(int i, int dim);

  PixelType border_pixel(const SamplerState& sampler) const;

  PixelType fetch_addressed(int x, int y, uint32_t level, AddressMode address_u,
                            AddressMode address_v,
                            const SamplerState& sampler) const;

  PixelType lerp_pixel(const PixelType& a, const PixelType& b, float t) const;
  PixelType bilerp_pixel(const PixelType& c00, const PixelType& c10,
                         const PixelType& c01, const PixelType& c11, float fx,
                         float fy) const;

  PixelType sample_level(float u, float v, uint32_t level, FilterMode filter,
                         const SamplerState& sampler) const;
};

using TextureRGBA8 = Texture2D<RGBA8>;

template <typename PixelType>
Texture2D<PixelType>::Texture2D() = default;

template <typename PixelType>
Texture2D<PixelType>::Texture2D(uint32_t width, uint32_t height,
                                uint32_t mip_levels) {
  allocate(width, height, mip_levels == 0 ? 1u : mip_levels);
}

template <typename PixelType>
Texture2D<PixelType>::Texture2D(const ImageType& base_level) {
  set_base_level(base_level);
}

template <typename PixelType>
Texture2D<PixelType>::Texture2D(const Texture2D& other) = default;

template <typename PixelType>
Texture2D<PixelType>::Texture2D(Texture2D&& other) noexcept = default;

template <typename PixelType>
Texture2D<PixelType>& Texture2D<PixelType>::operator=(const Texture2D& other) =
    default;

template <typename PixelType>
Texture2D<PixelType>& Texture2D<PixelType>::operator=(
    Texture2D&& other) noexcept = default;

template <typename PixelType>
Texture2D<PixelType>::~Texture2D() = default;

template <typename PixelType>
bool Texture2D<PixelType>::empty() const {
  return mip_levels_.empty() || width_ == 0 || height_ == 0;
}

template <typename PixelType>
uint32_t Texture2D<PixelType>::width() const {
  return width_;
}

template <typename PixelType>
uint32_t Texture2D<PixelType>::height() const {
  return height_;
}

template <typename PixelType>
uint32_t Texture2D<PixelType>::num_mip_levels() const {
  return static_cast<uint32_t>(mip_levels_.size());
}

template <typename PixelType>
bool Texture2D<PixelType>::has_mipmaps() const {
  return num_mip_levels() > 1;
}

template <typename PixelType>
uint32_t Texture2D<PixelType>::compute_full_mip_count(uint32_t w, uint32_t h) {
  if (w == 0 || h == 0) return 0;
  uint32_t levels = 1;
  while (w > 1 || h > 1) {
    w = w > 1 ? (w >> 1) : 1;
    h = h > 1 ? (h >> 1) : 1;
    ++levels;
  }
  return levels;
}

template <typename PixelType>
bool Texture2D<PixelType>::is_level_valid(uint32_t level) const {
  return level < mip_levels_.size();
}

template <typename PixelType>
std::pair<uint32_t, uint32_t> Texture2D<PixelType>::level_dimensions(
    uint32_t level) const {
  if (!is_level_valid(level)) return {0u, 0u};
  return {mip_levels_[level].width(), mip_levels_[level].height()};
}

template <typename PixelType>
const typename Texture2D<PixelType>::ImageType& Texture2D<PixelType>::mip(
    uint32_t level) const {
  return mip_levels_[level];
}

template <typename PixelType>
typename Texture2D<PixelType>::ImageType& Texture2D<PixelType>::mip(
    uint32_t level) {
  return mip_levels_[level];
}

template <typename PixelType>
const PixelType* Texture2D<PixelType>::data(uint32_t level) const {
  return mip_levels_[level].data();
}

template <typename PixelType>
PixelType* Texture2D<PixelType>::data(uint32_t level) {
  return mip_levels_[level].data();
}

template <typename PixelType>
void Texture2D<PixelType>::allocate(uint32_t w, uint32_t h,
                                    uint32_t mip_levels) {
  width_ = w;
  height_ = h;
  if (mip_levels == 0) mip_levels = 1;
  mip_levels_.clear();
  mip_levels_.reserve(mip_levels);

  uint32_t level_w = w;
  uint32_t level_h = h;
  for (uint32_t i = 0; i < mip_levels; ++i) {
    mip_levels_.emplace_back(level_w, level_h);
    if (level_w > 1) level_w >>= 1;
    if (level_h > 1) level_h >>= 1;
  }
}

template <typename PixelType>
void Texture2D<PixelType>::set_base_level(const ImageType& image) {
  width_ = image.width();
  height_ = image.height();
  mip_levels_.clear();
  mip_levels_.push_back(image);
}

template <typename PixelType>
void Texture2D<PixelType>::set_mip_chain(const std::vector<ImageType>& levels) {
  if (levels.empty()) {
    mip_levels_.clear();
    width_ = 0;
    height_ = 0;
    return;
  }
  mip_levels_ = levels;
  width_ = mip_levels_[0].width();
  height_ = mip_levels_[0].height();
}

template <typename PixelType>
void Texture2D<PixelType>::clear(const PixelType& value) {
  for (auto& img : mip_levels_) {
    uint32_t w = img.width();
    uint32_t h = img.height();
    for (uint32_t y = 0; y < h; ++y) {
      for (uint32_t x = 0; x < w; ++x) {
        img(x, y) = value;
      }
    }
  }
}

template <typename PixelType>
void Texture2D<PixelType>::generate_mipmaps(FilterMode /*filter*/) {
  if (empty()) return;
  const uint32_t full_levels = compute_full_mip_count(width_, height_);
  if (num_mip_levels() < full_levels) {
    // allocate missing levels
    uint32_t w = width_;
    uint32_t h = height_;
    mip_levels_.reserve(full_levels);
    for (uint32_t level = 1; level < full_levels; ++level) {
      w = std::max(1u, w >> 1);
      h = std::max(1u, h >> 1);
      mip_levels_.emplace_back(w, h);
    }
  }

  // Downsample level by level (simple box filter 2x2 with edge handling)
  for (uint32_t level = 1; level < num_mip_levels(); ++level) {
    const auto& src = mip_levels_[level - 1];
    auto& dst = mip_levels_[level];
    uint32_t sw = src.width();
    uint32_t sh = src.height();
    uint32_t dw = dst.width();
    uint32_t dh = dst.height();

    for (uint32_t y = 0; y < dh; ++y) {
      for (uint32_t x = 0; x < dw; ++x) {
        uint32_t sx = x * 2;
        uint32_t sy = y * 2;
        // Gather up to 4 samples within bounds
        uint32_t count = 0;
        if constexpr (std::is_same_v<PixelType, RGBA8>) {
          uint32_t r = 0, g = 0, b = 0, a = 0;
          for (uint32_t oy = 0; oy < 2; ++oy) {
            for (uint32_t ox = 0; ox < 2; ++ox) {
              uint32_t px = sx + ox;
              uint32_t py = sy + oy;
              if (px < sw && py < sh) {
                const RGBA8& c = src(px, py);
                r += c.r;
                g += c.g;
                b += c.b;
                a += c.a;
                ++count;
              }
            }
          }
          if (count == 0) count = 1;
          dst(x, y) = RGBA8(
              static_cast<uint8_t>(r / count), static_cast<uint8_t>(g / count),
              static_cast<uint8_t>(b / count), static_cast<uint8_t>(a / count));
        } else if constexpr (std::is_floating_point_v<PixelType>) {
          float sum = 0.0f;
          for (uint32_t oy = 0; oy < 2; ++oy) {
            for (uint32_t ox = 0; ox < 2; ++ox) {
              uint32_t px = sx + ox;
              uint32_t py = sy + oy;
              if (px < sw && py < sh) {
                sum += src(px, py);
                ++count;
              }
            }
          }
          if (count == 0) count = 1;
          dst(x, y) = static_cast<PixelType>(sum / static_cast<float>(count));
        } else {
          // Fallback: pick top-left sample
          uint32_t px = std::min(sx, sw - 1);
          uint32_t py = std::min(sy, sh - 1);
          dst(x, y) = src(px, py);
        }
      }
    }
  }
}

// ------------ Sampling helpers ------------

template <typename PixelType>
int Texture2D<PixelType>::wrap_repeat(int i, int dim) {
  if (dim <= 0) return 0;
  int m = i % dim;
  return m < 0 ? m + dim : m;
}

template <typename PixelType>
int Texture2D<PixelType>::wrap_mirror(int i, int dim) {
  if (dim <= 0) return 0;
  int period = dim * 2;
  int m = i % period;
  if (m < 0) m += period;
  if (m < dim) return m;
  return period - 1 - m;
}

template <typename PixelType>
int Texture2D<PixelType>::clamp_index(int i, int dim) {
  if (dim <= 0) return 0;
  if (i < 0) return 0;
  if (i >= dim) return dim - 1;
  return i;
}

template <typename PixelType>
PixelType Texture2D<PixelType>::border_pixel(
    const SamplerState& sampler) const {
  if constexpr (std::is_same_v<PixelType, RGBA8>) {
    return sampler.border_color;
  } else {
    return PixelType();
  }
}

template <typename PixelType>
PixelType Texture2D<PixelType>::fetch_addressed(
    int x, int y, uint32_t level, AddressMode address_u, AddressMode address_v,
    const SamplerState& sampler) const {
  if (!is_level_valid(level)) return PixelType();
  const auto& img = mip_levels_[level];
  int w = static_cast<int>(img.width());
  int h = static_cast<int>(img.height());

  auto resolve = [](int i, int dim, AddressMode mode) -> std::pair<bool, int> {
    switch (mode) {
      case AddressMode::ClampToEdge:
        return {true, Texture2D::clamp_index(i, dim)};
      case AddressMode::Repeat:
        return {true, Texture2D::wrap_repeat(i, dim)};
      case AddressMode::MirrorRepeat:
        return {true, Texture2D::wrap_mirror(i, dim)};
      case AddressMode::ClampToBorder:
      default:
        if (i < 0 || i >= dim) return {false, 0};
        return {true, i};
    }
  };

  auto [okx, rx] = resolve(x, w, address_u);
  auto [oky, ry] = resolve(y, h, address_v);
  if (!okx || !oky) return border_pixel(sampler);
  return img(static_cast<uint32_t>(rx), static_cast<uint32_t>(ry));
}

template <typename PixelType>
PixelType Texture2D<PixelType>::lerp_pixel(const PixelType& a,
                                           const PixelType& b, float t) const {
  float ft = std::clamp(t, 0.0f, 1.0f);
  if constexpr (std::is_same_v<PixelType, RGBA8>) {
    auto lerp_u8 = [ft](uint8_t x, uint8_t y) -> uint8_t {
      float xf = static_cast<float>(x);
      float yf = static_cast<float>(y);
      float rf = xf + (yf - xf) * ft;
      int ri = static_cast<int>(std::round(rf));
      return static_cast<uint8_t>(std::clamp(ri, 0, 255));
    };
    return RGBA8(lerp_u8(a.r, b.r), lerp_u8(a.g, b.g), lerp_u8(a.b, b.b),
                 lerp_u8(a.a, b.a));
  } else if constexpr (std::is_floating_point_v<PixelType>) {
    return static_cast<PixelType>(a + (b - a) * ft);
  } else {
    // Fallback: no interpolation support, return a
    (void)b;
    return a;
  }
}

template <typename PixelType>
PixelType Texture2D<PixelType>::bilerp_pixel(const PixelType& c00,
                                             const PixelType& c10,
                                             const PixelType& c01,
                                             const PixelType& c11, float fx,
                                             float fy) const {
  PixelType cx0 = lerp_pixel(c00, c10, fx);
  PixelType cx1 = lerp_pixel(c01, c11, fx);
  return lerp_pixel(cx0, cx1, fy);
}

// ------------ Sampling core ------------

template <typename PixelType>
PixelType Texture2D<PixelType>::sample_level(
    float u, float v, uint32_t level, FilterMode filter,
    const SamplerState& sampler) const {
  if (!is_level_valid(level)) return PixelType();

  const auto& img = mip_levels_[level];
  int w = static_cast<int>(img.width());
  int h = static_cast<int>(img.height());
  if (w <= 0 || h <= 0) return PixelType();

  float vv = (sampler.uv_origin == UVOrigin::BottomLeft) ? v : (1.0f - v);

  if (filter == FilterMode::Nearest) {
    float sx = u * static_cast<float>(w);
    float sy = vv * static_cast<float>(h);
    int ix = static_cast<int>(std::floor(sx));
    int iy = static_cast<int>(std::floor(sy));
    return fetch_addressed(ix, iy, level, sampler.address_u, sampler.address_v,
                           sampler);
  } else {
    // Bilinear
    float sx = u * static_cast<float>(w) - 0.5f;
    float sy = vv * static_cast<float>(h) - 0.5f;
    int ix0 = static_cast<int>(std::floor(sx));
    int iy0 = static_cast<int>(std::floor(sy));
    float fx = sx - static_cast<float>(ix0);
    float fy = sy - static_cast<float>(iy0);

    PixelType c00 = fetch_addressed(ix0 + 0, iy0 + 0, level, sampler.address_u,
                                    sampler.address_v, sampler);
    PixelType c10 = fetch_addressed(ix0 + 1, iy0 + 0, level, sampler.address_u,
                                    sampler.address_v, sampler);
    PixelType c01 = fetch_addressed(ix0 + 0, iy0 + 1, level, sampler.address_u,
                                    sampler.address_v, sampler);
    PixelType c11 = fetch_addressed(ix0 + 1, iy0 + 1, level, sampler.address_u,
                                    sampler.address_v, sampler);

    return bilerp_pixel(c00, c10, c01, c11, fx, fy);
  }
}

template <typename PixelType>
PixelType Texture2D<PixelType>::sample_uv(float u, float v,
                                          const SamplerState& sampler) const {
  if (empty()) return PixelType();
  FilterMode filter = sampler.mag_filter;
  uint32_t level = 0;
  return sample_level(u, v, level, filter, sampler);
}

template <typename PixelType>
PixelType Texture2D<PixelType>::sample_lod(float u, float v, float lod,
                                           const SamplerState& sampler) const {
  if (empty()) return PixelType();

  float eff_lod = lod + sampler.lod_bias;
  eff_lod = std::clamp(eff_lod, sampler.min_lod, sampler.max_lod);

  uint32_t last_level = num_mip_levels() > 0 ? num_mip_levels() - 1 : 0;

  if (sampler.mip_filter == MipFilterMode::None || last_level == 0) {
    return sample_level(u, v, 0u, sampler.mag_filter, sampler);
  }

  if (sampler.mip_filter == MipFilterMode::Nearest) {
    int level_idx = static_cast<int>(std::round(eff_lod));
    level_idx = std::clamp(level_idx, 0, static_cast<int>(last_level));
    FilterMode filter =
        (eff_lod <= 0.0f) ? sampler.mag_filter : sampler.min_filter;
    return sample_level(u, v, static_cast<uint32_t>(level_idx), filter,
                        sampler);
  }

  // Linear (trilinear) between two adjacent levels
  float lf = std::clamp(eff_lod, 0.0f, static_cast<float>(last_level));
  uint32_t l0 = static_cast<uint32_t>(std::floor(lf));
  uint32_t l1 = std::min(l0 + 1, last_level);
  float t = lf - static_cast<float>(l0);
  FilterMode filter0 = (lf <= 0.0f) ? sampler.mag_filter : sampler.min_filter;
  FilterMode filter1 = (lf < 1.0f) ? sampler.mag_filter : sampler.min_filter;

  PixelType s0 = sample_level(u, v, l0, filter0, sampler);
  PixelType s1 = sample_level(u, v, l1, filter1, sampler);
  return lerp_pixel(s0, s1, t);
}

template <typename PixelType>
PixelType Texture2D<PixelType>::sample_grad(float u, float v, float dudx,
                                            float dudy, float dvdx, float dvdy,
                                            const SamplerState& sampler) const {
  if (empty()) return PixelType();
  // Compute texture footprint using base level dimensions
  float w = static_cast<float>(width_);
  float h = static_cast<float>(height_);

  float dudx_tex = dudx * w;
  float dvdx_tex = dvdx * h;
  float dudy_tex = dudy * w;
  float dvdy_tex = dvdy * h;

  float len_x = std::sqrt(dudx_tex * dudx_tex + dvdx_tex * dvdx_tex);
  float len_y = std::sqrt(dudy_tex * dudy_tex + dvdy_tex * dvdy_tex);
  float rho = std::max(len_x, len_y);
  float lod = std::log2(std::max(rho, 1e-8f));

  return sample_lod(u, v, lod, sampler);
}

template <typename PixelType>
PixelType Texture2D<PixelType>::sample_texel(int x, int y,
                                             uint32_t level) const {
  if (!is_level_valid(level)) {
    return PixelType();
  }
  const auto& img = mip_levels_[level];
  if (x < 0 || y < 0) {
    return PixelType();
  }
  if (static_cast<uint32_t>(x) >= img.width() ||
      static_cast<uint32_t>(y) >= img.height()) {
    return PixelType();
  }
  return img(static_cast<uint32_t>(x), static_cast<uint32_t>(y));
}

}  // namespace core
}  // namespace soft_renderer