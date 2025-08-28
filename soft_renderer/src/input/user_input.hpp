#pragma once

#include <unordered_set>

#include "../core/math/vec.hpp"

namespace soft_renderer {
namespace input {

// Use a class enum for KeyCode to prevent naming conflicts and enhance type
// safety. We'll start by defining keys that might be useful for an
// OrbitController.
enum class KeyCode {
  // Rotation
  UP,
  DOWN,
  LEFT,
  RIGHT,

  // Panning
  W,
  A,
  S,
  D,

  // Zooming
  Q,  // Zoom in
  E,  // Zoom out

  // Numbers for controller switching
  KEY_1,
  KEY_2,
  KEY_3,
  KEY_4,

  // Modifiers
  L_SHIFT,
  L_CTRL,
};

// Define an enum for mouse buttons as well.
enum class MouseButton { LEFT, RIGHT, MIDDLE };

/**
 * @brief Represents the user input state for a single frame.
 *
 * This struct is "state-driven," meaning it doesn't care how events occurred,
 * but only records the current state of inputs for this frame.
 * In an offline rendering mode, this struct can be populated programmatically
 * to simulate user input.
 * In a real-time mode, it would be updated by the windowing system's
 * callbacks (like GLFW) at the beginning of each frame.
 */
struct UserInput {
  // --- Keyboard State ---
  // Use a set to store all currently pressed keys for efficient lookup.
  std::unordered_set<KeyCode> pressed_keys;

  // --- Mouse State ---
  // Even if you are only using the keyboard for now, these fields are crucial
  // for future expansion. A well-designed controller's rotation/panning logic
  // often depends only on the 2D vectors below, regardless of whether they
  // originate from a mouse or simulated keyboard input.

  // The change in mouse cursor position from the last frame to the current one.
  // This is the most important piece of information for smooth, analog-style
  // camera control.
  math::Vec2d mouse_delta{};

  // The amount the mouse wheel was scrolled.
  double scroll_delta = 0.0;

  // A set of currently pressed mouse buttons.
  std::unordered_set<MouseButton> pressed_mouse_buttons;
};

}  // namespace input
}  // namespace soft_renderer
