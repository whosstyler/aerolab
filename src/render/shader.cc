#include "render/shader.h"

#include <array>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "core/log.h"

namespace aerolab {
namespace {

// GL reports the buffer size including the terminating NUL; trim to the
// characters actually written so logs print cleanly.
std::string ShaderLog(GLuint shader) {
  GLint length = 0;
  glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
  std::string log(static_cast<std::size_t>(length > 1 ? length : 1), '\0');
  GLsizei written = 0;
  glGetShaderInfoLog(shader, length, &written, log.data());
  log.resize(static_cast<std::size_t>(written > 0 ? written : 0));
  return log;
}

std::string ProgramLog(GLuint program) {
  GLint length = 0;
  glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
  std::string log(static_cast<std::size_t>(length > 1 ? length : 1), '\0');
  GLsizei written = 0;
  glGetProgramInfoLog(program, length, &written, log.data());
  log.resize(static_cast<std::size_t>(written > 0 ? written : 0));
  return log;
}

// Returns 0 on failure.
GLuint CompileStage(GLenum stage, std::string_view source,
                    std::string_view name) {
  const GLuint shader = glCreateShader(stage);
  const GLchar* text = source.data();
  const GLint length = static_cast<GLint>(source.size());
  glShaderSource(shader, 1, &text, &length);
  glCompileShader(shader);
  GLint ok = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
  if (ok == 0) {
    LogError("{} {} shader failed to compile:\n{}", name,
             stage == GL_VERTEX_SHADER ? "vertex" : "fragment",
             ShaderLog(shader));
    glDeleteShader(shader);
    return 0;
  }
  return shader;
}

std::array<float, 9> ToFloats(const Mat3& m) {
  std::array<float, 9> result;
  for (int i = 0; i < 9; ++i) result[i] = static_cast<float>(m.elements()[i]);
  return result;
}

}  // namespace

std::optional<ShaderProgram> ShaderProgram::Create(
    std::string_view name, std::string_view vertex_source,
    std::string_view fragment_source) {
  const GLuint vertex = CompileStage(GL_VERTEX_SHADER, vertex_source, name);
  if (vertex == 0) return std::nullopt;
  const GLuint fragment =
      CompileStage(GL_FRAGMENT_SHADER, fragment_source, name);
  if (fragment == 0) {
    glDeleteShader(vertex);
    return std::nullopt;
  }

  const GLuint program = glCreateProgram();
  glAttachShader(program, vertex);
  glAttachShader(program, fragment);
  glLinkProgram(program);
  glDetachShader(program, vertex);
  glDetachShader(program, fragment);
  glDeleteShader(vertex);
  glDeleteShader(fragment);

  GLint ok = 0;
  glGetProgramiv(program, GL_LINK_STATUS, &ok);
  if (ok == 0) {
    LogError("{} program failed to link:\n{}", name, ProgramLog(program));
    glDeleteProgram(program);
    return std::nullopt;
  }
  return ShaderProgram(program);
}

ShaderProgram::~ShaderProgram() {
  if (program_ != 0) glDeleteProgram(program_);
}

ShaderProgram::ShaderProgram(ShaderProgram&& other) noexcept
    : program_(std::exchange(other.program_, 0)),
      locations_(std::move(other.locations_)) {}

ShaderProgram& ShaderProgram::operator=(ShaderProgram&& other) noexcept {
  if (this != &other) {
    if (program_ != 0) glDeleteProgram(program_);
    program_ = std::exchange(other.program_, 0);
    locations_ = std::move(other.locations_);
  }
  return *this;
}

void ShaderProgram::Use() const { glUseProgram(program_); }

GLint ShaderProgram::Location(std::string_view name) {
  const auto it = locations_.find(name);
  if (it != locations_.end()) return it->second;
  const std::string key(name);
  const GLint location = glGetUniformLocation(program_, key.c_str());
  locations_.emplace(key, location);
  return location;
}

void ShaderProgram::SetInt(std::string_view name, int value) {
  glUniform1i(Location(name), value);
}

void ShaderProgram::SetFloat(std::string_view name, float value) {
  glUniform1f(Location(name), value);
}

void ShaderProgram::SetVec2(std::string_view name, float x, float y) {
  glUniform2f(Location(name), x, y);
}

void ShaderProgram::SetVec3(std::string_view name, const Vec3& value) {
  glUniform3f(Location(name), static_cast<float>(value.x),
              static_cast<float>(value.y), static_cast<float>(value.z));
}

void ShaderProgram::SetColor3(std::string_view name, const Color& value) {
  glUniform3f(Location(name), value.r, value.g, value.b);
}

void ShaderProgram::SetColor4(std::string_view name, const Color& value) {
  glUniform4f(Location(name), value.r, value.g, value.b, value.a);
}

void ShaderProgram::SetMat3(std::string_view name, const Mat3& value) {
  const std::array<float, 9> floats = ToFloats(value);
  glUniformMatrix3fv(Location(name), 1, GL_FALSE, floats.data());
}

void ShaderProgram::SetMat4(std::string_view name, const Mat4& value) {
  const std::array<float, 16> floats = value.ToFloatArray();
  glUniformMatrix4fv(Location(name), 1, GL_FALSE, floats.data());
}

}  // namespace aerolab
