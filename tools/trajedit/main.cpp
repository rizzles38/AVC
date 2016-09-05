#include <cstdlib>
#include <iostream>
#include <string>

#include <epoxy/gl.h>
#include <GLFW/glfw3.h>

#include "window.h"

static const char* vert_shader_src =
  "#version 330\n"
  "\n"
  "in vec2 position;\n"
  "\n"
  "void main() {\n"
  "  gl_Position = vec4(position, 0.0, 1.0);\n"
  "}\n";

static const char* frag_shader_src =
  "#version 330\n"
  "\n"
  "out vec4 out_color;\n"
  "\n"
  "void main() {\n"
  "  out_color = vec4(1.0, 1.0, 1.0, 1.0);\n"
  "}\n";

int main(int argc, char* argv[]) {
  Window window;

  GLuint vao;
  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  float vertices[] = {
    0.0f, 0.5f,
    0.5f, -0.5f,
    -0.5f, -0.5f
  };

  GLuint vbo;
  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

  GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertex_shader, 1, &vert_shader_src, nullptr);
  glCompileShader(vertex_shader);
  GLint status;
  glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &status);
  if (status != GL_TRUE) {
    char buffer[512];
    glGetShaderInfoLog(vertex_shader, 512, nullptr, buffer);
    std::cout << "Vert shader compile failure:\n" << buffer << "\n";
    std::exit(EXIT_FAILURE);
  }

  GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragment_shader, 1, &frag_shader_src, nullptr);
  glCompileShader(fragment_shader);
  glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &status);
  if (status != GL_TRUE) {
    char buffer[512];
    glGetShaderInfoLog(fragment_shader, 512, nullptr, buffer);
    std::cout << "Vert shader compile failure:\n" << buffer << "\n";
    std::exit(EXIT_FAILURE);
  }

  GLuint shader_program = glCreateProgram();
  glAttachShader(shader_program, vertex_shader);
  glAttachShader(shader_program, fragment_shader);

  glBindFragDataLocation(shader_program, 0, "out_color");

  glLinkProgram(shader_program);

  glUseProgram(shader_program);

  GLint pos_attrib = glGetAttribLocation(shader_program, "position");
  glVertexAttribPointer(pos_attrib, 2, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(pos_attrib);

  while (!glfwWindowShouldClose(window)) {
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);

    glDrawArrays(GL_TRIANGLES, 0, 3);

    glfwSwapBuffers(window);
    glfwPollEvents();

    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
      glfwSetWindowShouldClose(window, GL_TRUE);
    }
  }

  glfwTerminate();
  return EXIT_SUCCESS;
}
