#include "input_manager.hpp"

#include <MiniFB.h>

namespace soft_renderer {
namespace input {

void InputManager::bind_to_window(platform::Window& window) {
  if (is_bound_) {
    return;
  }

  window.set_key_callback([this](int key, int mod, bool is_down) {
    on_key_event(key, mod, is_down);
  });

  window.set_mouse_button_callback([this](int button, int mod, bool is_down) {
    on_mouse_button_event(button, mod, is_down);
  });

  window.set_mouse_move_callback(
      [this](int x, int y) { on_mouse_move_event(x, y); });

  window.set_scroll_callback([this](int mod, float x_offset, float y_offset) {
    on_scroll_event(mod, x_offset, y_offset);
  });

  is_bound_ = true;
}

void InputManager::update_frame() {
  previous_input_ = current_input_;

  current_input_.mouse_delta = current_mouse_pos_ - last_mouse_pos_;
  last_mouse_pos_ = current_mouse_pos_;

  current_input_.scroll_delta = 0.0;
}

bool InputManager::was_key_just_pressed(KeyCode key) const {
  return current_input_.pressed_keys.count(key) > 0 &&
         previous_input_.pressed_keys.count(key) == 0;
}

bool InputManager::was_key_just_released(KeyCode key) const {
  return current_input_.pressed_keys.count(key) == 0 &&
         previous_input_.pressed_keys.count(key) > 0;
}

bool InputManager::was_mouse_button_just_pressed(MouseButton button) const {
  return current_input_.pressed_mouse_buttons.count(button) > 0 &&
         previous_input_.pressed_mouse_buttons.count(button) == 0;
}

bool InputManager::was_mouse_button_just_released(MouseButton button) const {
  return current_input_.pressed_mouse_buttons.count(button) == 0 &&
         previous_input_.pressed_mouse_buttons.count(button) > 0;
}

void InputManager::on_key_event(int platform_key, int mod, bool is_down) {
  KeyCode key_code = platform_key_to_keycode(platform_key);

  if (is_down) {
    current_input_.pressed_keys.insert(key_code);
  } else {
    current_input_.pressed_keys.erase(key_code);
  }
}

void InputManager::on_mouse_button_event(int platform_button, int mod,
                                         bool is_down) {
  MouseButton mouse_button = platform_button_to_mouse_button(platform_button);

  if (is_down) {
    current_input_.pressed_mouse_buttons.insert(mouse_button);
  } else {
    current_input_.pressed_mouse_buttons.erase(mouse_button);
  }
}

void InputManager::on_mouse_move_event(int x, int y) {
  current_mouse_pos_ =
      math::Vec2d(static_cast<double>(x), static_cast<double>(y));
}

void InputManager::on_scroll_event(int mod, float x_offset, float y_offset) {
  current_input_.scroll_delta = static_cast<double>(y_offset);
}

KeyCode InputManager::platform_key_to_keycode(int platform_key) const {
  switch (platform_key) {
    case KB_KEY_UP:
      return KeyCode::UP;
    case KB_KEY_DOWN:
      return KeyCode::DOWN;
    case KB_KEY_LEFT:
      return KeyCode::LEFT;
    case KB_KEY_RIGHT:
      return KeyCode::RIGHT;
    case KB_KEY_W:
      return KeyCode::W;
    case KB_KEY_A:
      return KeyCode::A;
    case KB_KEY_S:
      return KeyCode::S;
    case KB_KEY_D:
      return KeyCode::D;
    case KB_KEY_Q:
      return KeyCode::Q;
    case KB_KEY_E:
      return KeyCode::E;
    case KB_KEY_1:
      return KeyCode::KEY_1;
    case KB_KEY_2:
      return KeyCode::KEY_2;
    case KB_KEY_3:
      return KeyCode::KEY_3;
    case KB_KEY_4:
      return KeyCode::KEY_4;
    case KB_KEY_LEFT_SHIFT:
      return KeyCode::L_SHIFT;
    case KB_KEY_LEFT_CONTROL:
      return KeyCode::L_CTRL;
    default:
      return KeyCode::UP;
  }
}

MouseButton InputManager::platform_button_to_mouse_button(
    int platform_button) const {
  switch (platform_button) {
    case MOUSE_BTN_1:
      return MouseButton::LEFT;
    case MOUSE_BTN_2:
      return MouseButton::RIGHT;
    case MOUSE_BTN_3:
      return MouseButton::MIDDLE;
    default:
      return MouseButton::LEFT;
  }
}

}  // namespace input
}  // namespace soft_renderer