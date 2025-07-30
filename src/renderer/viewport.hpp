#pragma once

#include "camera.hpp"
#include "pipeline_types.hpp"

namespace soft_renderer {
namespace renderer {

// Viewport aggregates a camera and the context for a specific rendering
// operation. It does not own the camera.
class Viewport {
 public:
  Camera* camera;

  int width;
  int height;

  double near_plane;
  double far_plane;

  Viewport(Camera* cam, int w, int h)
      : camera(cam), width(w), height(h), near_plane(0.1), far_plane(1000.0) {}

  void set_size(int w, int h) {
    width = w;
    height = h;
  }

  // Convenience function to get the view matrix from the associated camera.
  Mat4d get_view_matrix() const {
    if (!camera) {
      return Mat4d::identity();
    }
    return camera->get_view_matrix();
  }

  // Convenience function to get the projection matrix.
  // It passes its own context (aspect ratio, near, far) to the camera.
  Mat4d get_projection_matrix() const {
    if (!camera || height == 0) {
      return Mat4d::identity();
    }
    double aspect_ratio =
        static_cast<double>(width) / static_cast<double>(height);
    return camera->get_projection_matrix(aspect_ratio, near_plane, far_plane);
  }
};

class ViewportTransform {
 public:
  ViewportTransform(uint32_t x, uint32_t y, uint32_t width, uint32_t height)
      : x_(x), y_(y), width_(width), height_(height) {}

  ViewportTransform(uint32_t width, uint32_t height)
      : x_(0), y_(0), width_(width), height_(height) {}

  ScreenCoord ndc_to_screen(const Vec3f& ndc) const {
    int screen_x = x_ + (ndc.x() + 1.0f) * 0.5f * width_;
    int screen_y = y_ + (ndc.y() + 1.0f) * 0.5f * height_;
    float depth = (ndc.z() + 1.0f) * 0.5f;
    return {screen_x, screen_y, depth};
  }

 private:
  uint32_t x_;
  uint32_t y_;
  uint32_t width_;
  uint32_t height_;
};

}  // namespace renderer
}  // namespace soft_renderer
