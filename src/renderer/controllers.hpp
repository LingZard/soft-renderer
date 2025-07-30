#pragma once

#include "../input/user_input.hpp"
#include "camera.hpp"

// TODO

namespace soft_renderer {
namespace renderer {

using namespace soft_renderer::math;
using namespace soft_renderer::input;

class OrbitController : public ICameraController {
 public:
  void update(Camera& camera, double delta_time,
              const UserInput& input) override {
    // TODO
  };
};

}  // namespace renderer
}  // namespace soft_renderer