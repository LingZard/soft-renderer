#pragma once

#include <fstream>
#include <ios>
#include <iostream>
#include <vector>

#include "../core/image.hpp"

namespace soft_renderer {
namespace io {

using namespace soft_renderer::core;

// This pragma ensures that the compiler packs the struct members without any
// alignment padding. This is crucial because we're going to read this struct
// directly from a binary file, and we need its memory layout to match the TGA
// file format specification exactly.
#pragma pack(push, 1)
struct TGAHeader {
  uint8_t id_length;
  uint8_t colormap_type;
  uint8_t image_type;
  uint16_t colormap_origin;
  uint16_t colormap_length;
  uint8_t colormap_depth;
  uint16_t x_origin;
  uint16_t y_origin;
  uint16_t width;
  uint16_t height;
  uint8_t bits_per_pixel;
  uint8_t image_descriptor;
};
#pragma pack(pop)

// Reads a TGA image from a file.
// This function currently supports uncompressed 24-bit (RGB) and 32-bit (RGBA)
// TGA files.
inline Image<RGBA8> read_tga_image(const std::string& filename) {
  std::ifstream in(filename, std::ios::binary);
  if (!in) {
    // It's good practice to let the user know if something went wrong.
    std::cerr << "Error: Could not open file for reading: " << filename
              << std::endl;
    return {};
  }

  TGAHeader header;
  in.read(reinterpret_cast<char*>(&header), sizeof(header));
  if (!in) {
    std::cerr << "Error: Could not read TGA header from " << filename
              << std::endl;
    return {};
  }

  // We're keeping it simple and only supporting uncompressed, true-color
  // images.
  if (header.image_type != 2) {
    std::cerr
        << "Error: Only uncompressed TGA images are supported. Image type: "
        << (int)header.image_type << std::endl;
    return {};
  }

  if (header.bits_per_pixel != 24 && header.bits_per_pixel != 32) {
    std::cerr << "Error: Unsupported TGA pixel depth: "
              << (int)header.bits_per_pixel
              << ". Only 24 or 32 bits per pixel are supported." << std::endl;
    return {};
  }

  Image<RGBA8> image(header.width, header.height);
  // Skip over the image ID field if it exists.
  in.seekg(header.id_length, std::ios_base::cur);

  uint32_t n_pixels = header.width * header.height;
  uint8_t bytes_per_pixel = header.bits_per_pixel / 8;
  std::vector<uint8_t> buffer(n_pixels * bytes_per_pixel);
  in.read(reinterpret_cast<char*>(buffer.data()), buffer.size());
  if (!in) {
    std::cerr << "Error: Could not read pixel data from " << filename
              << std::endl;
    return {};
  }

  // TGA stores colors in BGR(A) order, so we need to swizzle them to RGB(A)
  // when loading.
  for (uint32_t i = 0; i < n_pixels; ++i) {
    uint8_t b = buffer[i * bytes_per_pixel + 0];
    uint8_t g = buffer[i * bytes_per_pixel + 1];
    uint8_t r = buffer[i * bytes_per_pixel + 2];
    uint8_t a = (bytes_per_pixel == 4) ? buffer[i * bytes_per_pixel + 3] : 255;
    image.data()[i] = RGBA8(r, g, b, a);
  }

  // The TGA spec is a bit weird. The 5th bit of image_descriptor tells us the
  // origin. If it's 0, the origin is at the bottom-left. Our Image class
  // assumes a top-left origin, so we need to flip the image vertically.
  if (!(header.image_descriptor & 0x20)) {
    Image<RGBA8> flipped_image(header.width, header.height);
    for (uint32_t y = 0; y < header.height; ++y) {
      for (uint32_t x = 0; x < header.width; ++x) {
        flipped_image(x, y) = image(x, header.height - y - 1);
      }
    }
    return flipped_image;
  }

  return image;
}

inline bool write_tga_image(const std::string& filename,
                            const Image<RGBA8>& image,
                            bool origin_bottom_left = true) {
  std::ofstream out(filename, std::ios::binary);
  if (!out) {
    std::cerr << "Error: Cannot open file for writing: " << filename
              << std::endl;
    return false;
  }

  TGAHeader header{};
  header.image_type = 2;  // Uncompressed, true-color image
  header.width = image.width();
  header.height = image.height();
  header.bits_per_pixel = 32;
  // Bit 5 (0x20) being 0 means bottom-left origin, 1 means top-left.
  // This is the opposite of how we read it, because we're *writing* the flag.
  header.image_descriptor = origin_bottom_left ? 0x00 : 0x20;

  out.write(reinterpret_cast<const char*>(&header), sizeof(header));
  if (!out) {
    std::cerr << "Error: Could not write TGA header to " << filename
              << std::endl;
    return false;
  }

  std::vector<uint8_t> row_buffer(image.width() * 4);
  for (uint32_t y = 0; y < image.height(); ++y) {
    // If we're writing in bottom-left origin mode, we need to process rows
    // from the last one to the first one.
    uint32_t row_idx = origin_bottom_left ? (image.height() - 1 - y) : y;
    const RGBA8* row_ptr = image.data() + row_idx * image.width();

    // Swizzle from RGBA back to BGRA for the file format.
    for (uint32_t x = 0; x < image.width(); ++x) {
      row_buffer[x * 4 + 0] = row_ptr[x].b;
      row_buffer[x * 4 + 1] = row_ptr[x].g;
      row_buffer[x * 4 + 2] = row_ptr[x].r;
      row_buffer[x * 4 + 3] = row_ptr[x].a;
    }
    out.write(reinterpret_cast<const char*>(row_buffer.data()),
              row_buffer.size());
    if (!out) {
      std::cerr << "Error writing pixel data to " << filename << std::endl;
      return false;
    }
  }

  return true;
}
}  // namespace io
}  // namespace soft_renderer