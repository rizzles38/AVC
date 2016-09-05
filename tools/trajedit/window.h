#pragma once

#include <epoxy/gl.h>
#include <GLFW/glfw3.h>

#include <string>

class Window {
public:
  Window();
  ~Window();

  bool shouldClose();
  void setShouldClose(bool should_close);

  int width() const { return width_; }
  int height() const { return height_; }
  std::string title() const { return title_; }

private:
  GLFWwindow* window_;
  int width_;
  int height_;
  std::string title_;
};
