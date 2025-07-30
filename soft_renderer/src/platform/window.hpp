#pragma once

#include <functional>
#include <string>

#include "../core/framebuffer.hpp"

struct mfb_window;

namespace soft_renderer {
namespace platform {

// Mouse button constants (matching MiniFB values)
namespace MouseButton {
constexpr int LEFT = 1;
constexpr int MIDDLE = 2;
constexpr int RIGHT = 3;
}  // namespace MouseButton

// Key modifier constants (matching MiniFB values)
namespace KeyModifier {
constexpr int SHIFT = 1;
constexpr int CTRL = 2;
constexpr int ALT = 4;
constexpr int SUPER = 8;
}  // namespace KeyModifier

class Window {
 public:
  Window(const std::string& title, int width, int height);
  ~Window();

  Window(const Window&) = delete;
  Window& operator=(const Window&) = delete;
  Window(Window&&) = delete;
  Window& operator=(Window&&) = delete;

  void main_loop(std::function<void(core::FrameBuffer& framebuffer)> update_fn);

  // User-facing API
  void set_key_callback(
      std::function<void(int key, int mod, bool is_down)> callback);
  void set_mouse_button_callback(
      std::function<void(int button, int mod, bool is_down)> callback);
  void set_mouse_move_callback(std::function<void(int x, int y)> callback);
  void set_scroll_callback(
      std::function<void(int mod, float x_offset, float y_offset)> callback);

  // Helper method to get current mouse position
  void get_mouse_position(int& x, int& y) const;

  // Accessor for callbacks (for C adapter functions)
  const std::function<void(int, int, bool)>& get_key_handler() const {
    return key_handler_;
  }
  const std::function<void(int, int, bool)>& get_mouse_button_handler() const {
    return mouse_button_handler_;
  }
  const std::function<void(int, int)>& get_mouse_move_handler() const {
    return mouse_move_handler_;
  }
  const std::function<void(int, float, float)>& get_scroll_handler() const {
    return scroll_handler_;
  }

 private:
  struct mfb_window* window_;
  int width_;
  int height_;
  core::FrameBuffer framebuffer_;

  // User callbacks
  std::function<void(int, int, bool)> key_handler_{nullptr};
  std::function<void(int, int, bool)> mouse_button_handler_{nullptr};
  std::function<void(int, int)> mouse_move_handler_{nullptr};
  std::function<void(int, float, float)> scroll_handler_{nullptr};
};

}  // namespace platform
}  // namespace soft_renderer
