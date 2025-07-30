#include "window.hpp"

#include <MiniFB.h>

#include <stdexcept>

// Forward declarations for C adapter functions
extern "C" {
static void key_adapter(struct mfb_window* window, mfb_key key, mfb_key_mod mod,
                        bool is_down);
static void mouse_button_adapter(struct mfb_window* window,
                                 mfb_mouse_button button, mfb_key_mod mod,
                                 bool is_down);
static void mouse_move_adapter(struct mfb_window* window, int x, int y);
static void scroll_adapter(struct mfb_window* window, mfb_key_mod mod,
                           float x_offset, float y_offset);
}

namespace soft_renderer {
namespace platform {

Window::Window(const std::string& title, int width, int height)
    : width_(width), height_(height), framebuffer_(width, height) {
  window_ = mfb_open_ex(title.c_str(), width, height, WF_RESIZABLE);
  if (!window_) {
    throw std::runtime_error("Failed to create MiniFB window.");
  }
  mfb_set_user_data(window_, this);
}

Window::~Window() {
  if (window_) {
    mfb_close(window_);
  }
}

void Window::main_loop(
    std::function<void(core::FrameBuffer& framebuffer)> update_fn) {
  if (!update_fn) {
    return;
  }

  do {
    update_fn(framebuffer_);

    int state = mfb_update_ex(window_,
                              reinterpret_cast<void*>(const_cast<core::RGBA8*>(
                                  framebuffer_.color_buffer().data())),
                              width_, height_);

    if (state < 0) {
      break;
    }
  } while (mfb_wait_sync(window_));
}

void Window::set_key_callback(std::function<void(int, int, bool)> callback) {
  key_handler_ = std::move(callback);
  mfb_set_keyboard_callback(window_, key_adapter);
}

void Window::set_mouse_button_callback(
    std::function<void(int, int, bool)> callback) {
  mouse_button_handler_ = std::move(callback);
  mfb_set_mouse_button_callback(window_, mouse_button_adapter);
}

void Window::set_mouse_move_callback(std::function<void(int, int)> callback) {
  mouse_move_handler_ = std::move(callback);
  mfb_set_mouse_move_callback(window_, mouse_move_adapter);
}

void Window::set_scroll_callback(
    std::function<void(int, float, float)> callback) {
  scroll_handler_ = std::move(callback);
  mfb_set_mouse_scroll_callback(window_, scroll_adapter);
}

void Window::get_mouse_position(int& x, int& y) const {
  x = mfb_get_mouse_x(window_);
  y = mfb_get_mouse_y(window_);
}

}  // namespace platform
}  // namespace soft_renderer

extern "C" {
static void key_adapter(struct mfb_window* window, mfb_key key, mfb_key_mod mod,
                        bool is_down) {
  if (auto* win = static_cast<soft_renderer::platform::Window*>(
          mfb_get_user_data(window))) {
    const auto& handler = win->get_key_handler();
    if (handler) {
      handler(static_cast<int>(key), static_cast<int>(mod), is_down);
    }
  }
}

static void mouse_button_adapter(struct mfb_window* window,
                                 mfb_mouse_button button, mfb_key_mod mod,
                                 bool is_down) {
  if (auto* win = static_cast<soft_renderer::platform::Window*>(
          mfb_get_user_data(window))) {
    const auto& handler = win->get_mouse_button_handler();
    if (handler) {
      handler(static_cast<int>(button), static_cast<int>(mod), is_down);
    }
  }
}

static void mouse_move_adapter(struct mfb_window* window, int x, int y) {
  if (auto* win = static_cast<soft_renderer::platform::Window*>(
          mfb_get_user_data(window))) {
    const auto& handler = win->get_mouse_move_handler();
    if (handler) {
      handler(x, y);
    }
  }
}

static void scroll_adapter(struct mfb_window* window, mfb_key_mod mod,
                           float x_offset, float y_offset) {
  if (auto* win = static_cast<soft_renderer::platform::Window*>(
          mfb_get_user_data(window))) {
    const auto& handler = win->get_scroll_handler();
    if (handler) {
      handler(static_cast<int>(mod), x_offset, y_offset);
    }
  }
}
}