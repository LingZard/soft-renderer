#pragma once

#include <algorithm>
#include <cmath>

#include "camera.hpp"

namespace soft_renderer {
namespace camera {

using namespace soft_renderer::math;

class OrbitController : public ICameraController {
 private:
  Vec3d target_ = Vec3d(0.0, 0.0, 0.0);
  double distance_ = 10.0;
  double min_distance_ = 0.1;
  double max_distance_ = 100.0;
  bool is_initialized_ = false;

  double azimuth_ = 0.0;
  double elevation_ = 0.0;
  double min_elevation_ = -M_PI / 2 + 0.01;
  double max_elevation_ = M_PI / 2 - 0.01;

  double rotate_sensitivity_ = 0.005;
  double zoom_sensitivity_ = 1.0;
  double pan_sensitivity_ = 0.01;

 public:
  OrbitController(const Vec3d& target = Vec3d(0.0, 0.0, 0.0),
                  double distance = 10.0)
      : target_(target), distance_(distance) {}

  void update(Camera& camera, double delta_time,
              const UserInput& input) override {
    if (!is_initialized_) {
      Vec3d offset = camera.get_position() - target_;
      distance_ = offset.norm();
      if (distance_ > 1e-6) {
        elevation_ = asin(offset.y() / distance_);
        azimuth_ = atan2(offset.x(), offset.z());
      }
      is_initialized_ = true;
    }

    bool is_right_mouse_pressed =
        input.pressed_mouse_buttons.count(MouseButton::RIGHT);
    bool is_left_mouse_pressed =
        input.pressed_mouse_buttons.count(MouseButton::LEFT);

    // Compute camera basis from current spherical parameters for stable panning
    double cos_elev_pan = cos(elevation_);
    Vec3d cam_pos_pan =
        target_ + Vec3d(distance_ * cos_elev_pan * sin(azimuth_),
                        distance_ * sin(elevation_),
                        distance_ * cos_elev_pan * cos(azimuth_));
    Vec3d pan_forward = (target_ - cam_pos_pan).normalized();
    Vec3d pan_right = pan_forward.cross(kWorldUp).normalized();
    Vec3d pan_up = pan_right.cross(pan_forward);

    if (input.mouse_delta.norm() > 0) {
      if (is_right_mouse_pressed) {
        // Pan
        double pan_x = input.mouse_delta.x() * pan_sensitivity_ * distance_;
        double pan_y = input.mouse_delta.y() * pan_sensitivity_ * distance_;

        target_ += pan_right * pan_x - pan_up * pan_y;
      } else if (is_left_mouse_pressed) {
        // Rotate
        azimuth_ += input.mouse_delta.x() * rotate_sensitivity_;
        elevation_ -= input.mouse_delta.y() * rotate_sensitivity_;

        elevation_ = std::clamp(elevation_, min_elevation_, max_elevation_);

        azimuth_ = fmod(azimuth_, 2.0 * M_PI);
        if (azimuth_ < 0) azimuth_ += 2.0 * M_PI;
      }
    }

    if (std::abs(input.scroll_delta) > 0) {
      distance_ *= (1.0 - input.scroll_delta * zoom_sensitivity_ * 0.1);
      distance_ = std::clamp(distance_, min_distance_, max_distance_);
    }

    double cos_elev = cos(elevation_);
    Vec3d camera_position =
        target_ + Vec3d(distance_ * cos_elev * sin(azimuth_),
                        distance_ * sin(elevation_),
                        distance_ * cos_elev * cos(azimuth_));

    Vec3d forward = (target_ - camera_position).normalized();
    Vec3d right = forward.cross(kWorldUp).normalized();
    Vec3d up = right.cross(forward);

    camera.set_position(camera_position);
    camera.look_at(target_, kWorldUp);
  }

  void set_target(const Vec3d& target) { target_ = target; }
  void set_distance(double distance) {
    distance_ = std::clamp(distance, min_distance_, max_distance_);
  }
  void set_distance_range(double min_dist, double max_dist) {
    min_distance_ = min_dist;
    max_distance_ = max_dist;
    distance_ = std::clamp(distance_, min_distance_, max_distance_);
  }

  const Vec3d& get_target() const { return target_; }
  double get_distance() const { return distance_; }
};

}  // namespace camera
}  // namespace soft_renderer