#pragma once

#include <cmath>
#include <memory>
#include <numbers>

#include "../core/math/mat.hpp"
#include "../core/math/quat.hpp"
#include "../core/math/vec.hpp"
#include "../input/user_input.hpp"

namespace soft_renderer {
namespace camera {

using namespace soft_renderer::math;
using namespace soft_renderer::input;

const Vec3d kWorldUp = Vec3d(0.0, 1.0, 0.0);

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
// This is done to maintain compatibility with typical renderer pipelines.
class Camera;

class ICameraController {
 public:
  virtual ~ICameraController() = default;
  virtual void update(Camera& camera, double delta_time,
                      const UserInput& input) = 0;
};

class Camera {
 protected:
  // In an ECS, this would typically be a Transform component. In OOP, it could
  // be composition or inheritance. For this learning project, defining it
  // directly in the class is reasonable, especially since this is more of a
  // library than a large-scale application.
  // We store the camera-to-world transformation.
  Vec3d position_;
  UnitQuatd orientation_;
  std::unique_ptr<ICameraController> controller_;

 public:
  Camera(const Vec3d& position, const UnitQuatd& orientation)
      : position_(position), orientation_(orientation), controller_(nullptr) {}

  Mat4d get_view_matrix() const {
    // Note for learning: The Camera intentionally stores its pose in a
    // CV-style convention (Y-down, Z-out). This method converts it to the
    // standard OpenGL view matrix (world-to-camera).
    UnitQuatd q_world2cv = orientation_.inverse();
    Vec3d t_world2cv = -(q_world2cv.rotate(position_));
    // The CV-to-GL conversion is a 180-degree rotation around the X-axis.
    UnitQuatd q_cv2gl = UnitQuatd(Vec3d(1.0, 0.0, 0.0), std::numbers::pi);
    UnitQuatd q_world2gl = q_cv2gl * q_world2cv;
    Vec3d t_world2gl = q_cv2gl.rotate(t_world2cv);
    Mat3d rot = q_world2gl.to_rotation_matrix();
    Mat4d view_matrix = Mat4d::identity();
    view_matrix.set_col(0, Vec4d(rot.col(0), 0.0));
    view_matrix.set_col(1, Vec4d(rot.col(1), 0.0));
    view_matrix.set_col(2, Vec4d(rot.col(2), 0.0));
    view_matrix.set_col(3, Vec4d(t_world2gl, 1.0));
    return view_matrix;
  };

  virtual Mat4d get_projection_matrix(double aspect_ratio, double near,
                                      double far) const = 0;

  void set_controller(std::unique_ptr<ICameraController> controller) {
    controller_ = std::move(controller);
  }

  void handle_input(double delta_time, const UserInput& input) {
    if (controller_) {
      controller_->update(*this, delta_time, input);
    }
  }

  void rotate(const UnitQuatd& q) { orientation_ = q * orientation_; }
  void set_orientation(const UnitQuatd& orientation) {
    orientation_ = orientation;
  }
  void set_position(const Vec3d& position) { position_ = position; }
  void translate(const Vec3d& delta) { position_ += delta; }

  void look_at(const Vec3d& target, const Vec3d& up = kWorldUp) {
    Mat4d look_at_matrix = soft_renderer::math::look_at(position_, target, up);
    orientation_ = UnitQuatd::from_rotation_matrix(look_at_matrix.rotation());
  }

  virtual void zoom(double factor) {
    Vec3d forward = orientation_.rotate(Vec3d(0, 0, -1));
    position_ += forward * factor;
  }

  const Vec3d& get_position() const { return position_; }
  const UnitQuatd& get_orientation() const { return orientation_; }
};

class PerspectiveCamera : public Camera {
 private:
  double fov_y_rad_;

 public:
  PerspectiveCamera(const Vec3d& position, const UnitQuatd& orientation,
                    double fov_y_rad)
      : Camera(position, orientation) {
    fov_y_rad_ = fov_y_rad;
  }

  Mat4d get_projection_matrix(double aspect_ratio, double near,
                              double far) const override {
    return perspective(aspect_ratio, fov_y_rad_, near, far);
  }

  void zoom(double factor) override {
    Vec3d forward = orientation_.rotate(Vec3d(0, 0, 1));
    position_ += forward * factor;
  }
};

class OrthographicCamera : public Camera {
 private:
  double left_, right_, bottom_, top_;

 public:
  OrthographicCamera(const Vec3d& position, const UnitQuatd& orientation,
                     double left, double right, double bottom, double top)
      : Camera(position, orientation) {
    left_ = left;
    right_ = right;
    bottom_ = bottom;
    top_ = top;
  }

  Mat4d get_projection_matrix(double aspect_ratio, double near,
                              double far) const override {
    return orthographic(left_, right_, bottom_, top_, near, far);
  }

  void zoom(double factor) override {
    double scale = 1.0 + factor;
    if (scale > 0.01) {
      left_ *= scale;
      right_ *= scale;
      bottom_ *= scale;
      top_ *= scale;
    }
  }
};

}  // namespace camera
}  // namespace soft_renderer