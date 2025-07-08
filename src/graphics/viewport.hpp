#pragma once

#include "camera.hpp"

namespace soft_renderer {
namespace graphics {

// Viewport aggregates a camera and the context for a specific rendering
// operation. It does not own the camera.
class Viewport {
 public:
  // Pointer to the camera to use (does not own it).
  Camera* camera;

  // Render target dimensions.
  int width;
  int height;

  // Frustum planes.
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

}  // namespace graphics
}  // namespace soft_renderer
