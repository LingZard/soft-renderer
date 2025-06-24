#pragma once

#include "../math/mat.hpp"
#include "../math/quat.hpp"
#include "../math/vec.hpp"

namespace soft_renderer {
namespace graphics {

using namespace soft_renderer::math;

// Note on coordinate systems (for learning purposes):
// This camera implementation uses a coordinate system convention similar to
// computer vision (CV) libraries like OpenCV for its internal position and
// orientation representation:
//   - Z-axis points outwards from the screen
//   - Y-axis points downwards
//   - X-axis points to the right
// This corresponds to an image origin at the top-left corner.
//
// However, the final view and projection matrices produced will conform to the
// standard OpenGL convention (right-handed system):
//   - Z-axis points inwards (into the screen)
//   - Y-axis points upwards
//   - X-axis points to the right
// This is done to maintain compatibility with typical graphics pipelines.
class Camera;

class ICameraController {};

class Camera {
 private:
  Vec3f position_;
};

class PerspectiveCamera : public Camera {};

}  // namespace graphics
}  // namespace soft_renderer