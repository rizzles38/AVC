#include "window.h"

#include <cstdlib>
#include <iostream>

namespace {

void errorCallback(int error, const char* description) {
  std::cout << "GLFW Error [" << error << "]: " << description << "\n";
  std::exit(EXIT_FAILURE);
}

} // namespace

Window::Window()
  : window_(nullptr),
    width_(1280),
    height(720),
    title_("Trajectory Editor") {
  glfwSetErrorCallback(errorCallback);
  glfwInit();

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
  glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
  glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

  window_ = glfwCreateWindow(width_, height_, title_.c_str(), nullptr, nullptr);

  glfwMakeContextCurrent(window_);
}

Window::~Window() {
  glfwDestroyWindow(window_);
  glfwTerminate();
}

bool Window::shouldClose() {
  return glfwWindowShouldClose(window_);
}

void Window::setShouldClose(bool should_close) {
  bool val = (should_close) ? GLFW_TRUE : GLFW_FALSE;
  glfwSetWindowShouldClose(window_, val);
}
