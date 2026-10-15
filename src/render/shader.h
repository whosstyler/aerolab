#ifndef AEROLAB_RENDER_SHADER_H_
#define AEROLAB_RENDER_SHADER_H_

#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>

#include "geometry/color.h"
#include "math/mat3.h"
#include "math/mat4.h"
#include "math/vec3.h"
#include "render/gl.h"

namespace aerolab {

// Owns a linked GLSL program. Move-only; the GL object is deleted with it.
class ShaderProgram {
 public:
  // Compiles and links the program. Returns nullopt (after logging the
  // compiler output with `name`) on failure.
  static std::optional<ShaderProgram> Create(std::string_view name,
                                             std::string_view vertex_source,
                                             std::string_view fragment_source);

  ShaderProgram() = default;
  ~ShaderProgram();
  ShaderProgram(ShaderProgram&& other) noexcept;
  ShaderProgram& operator=(ShaderProgram&& other) noexcept;
  ShaderProgram(const ShaderProgram&) = delete;
  ShaderProgram& operator=(const ShaderProgram&) = delete;

  bool valid() const { return program_ != 0; }
  GLuint id() const { return program_; }

  void Use() const;

  // Uniform setters. The program must be in use. Unknown names are ignored
  // (GLSL compilers strip unused uniforms).
  void SetInt(std::string_view name, int value);
  void SetFloat(std::string_view name, float value);
  void SetVec2(std::string_view name, float x, float y);
  void SetVec3(std::string_view name, const Vec3& value);
  void SetColor3(std::string_view name, const Color& value);
  void SetColor4(std::string_view name, const Color& value);
  void SetMat3(std::string_view name, const Mat3& value);
  void SetMat4(std::string_view name, const Mat4& value);

 private:
  explicit ShaderProgram(GLuint program) : program_(program) {}

  GLint Location(std::string_view name);

  // Transparent hashing lets lookups use string_view without allocating.
  struct StringHash {
    using is_transparent = void;
    std::size_t operator()(std::string_view s) const {
      return std::hash<std::string_view>{}(s);
    }
  };

  GLuint program_ = 0;
  std::unordered_map<std::string, GLint, StringHash, std::equal_to<>>
      locations_;
};

}  // namespace aerolab

#endif  // AEROLAB_RENDER_SHADER_H_
