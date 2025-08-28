#pragma once

#include "../platform/window.hpp"
#include "user_input.hpp"

namespace soft_renderer {
namespace input {

class InputManager {
 private:
  UserInput current_input_;
  UserInput previous_input_;

  math::Vec2d last_mouse_pos_{0.0, 0.0};
  math::Vec2d current_mouse_pos_{0.0, 0.0};

  bool is_bound_ = false;

  KeyCode platform_key_to_keycode(int platform_key) const;
  MouseButton platform_button_to_mouse_button(int platform_button) const;

 public:
  InputManager() = default;
  ~InputManager() = default;

  InputManager(const InputManager&) = delete;
  InputManager& operator=(const InputManager&) = delete;

  void bind_to_window(platform::Window& window);

  void update_frame();

  const UserInput& get_current_input() const { return current_input_; }

  const UserInput& get_previous_input() const { return previous_input_; }

  bool was_key_just_pressed(KeyCode key) const;
  bool was_key_just_released(KeyCode key) const;
  bool was_mouse_button_just_pressed(MouseButton button) const;
  bool was_mouse_button_just_released(MouseButton button) const;

 private:
  void on_key_event(int platform_key, int mod, bool is_down);
  void on_mouse_button_event(int platform_button, int mod, bool is_down);
  void on_mouse_move_event(int x, int y);
  void on_scroll_event(int mod, float x_offset, float y_offset);
};

}  // namespace input
}  // namespace soft_renderer