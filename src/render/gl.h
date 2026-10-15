#ifndef AEROLAB_RENDER_GL_H_
#define AEROLAB_RENDER_GL_H_

// Minimal OpenGL 3.3 core-profile API surface used by the renderer.
//
// The engine loads its own function pointers rather than depending on a
// generated loader. Every declaration lives in namespace `aerolab`, so the
// names never collide with C symbols exported by a system GL library. Do not
// include a system or SDL OpenGL header in the same translation unit: those
// headers define the GL_* constants below as macros.

#include <cstddef>
#include <cstdint>
#include <string_view>

#if defined(_WIN32) && !defined(_WIN64)
#define AEROLAB_GL_APIENTRY __stdcall
#else
#define AEROLAB_GL_APIENTRY
#endif

namespace aerolab {

using GLenum = unsigned int;
using GLboolean = unsigned char;
using GLbitfield = unsigned int;
using GLint = int;
using GLuint = unsigned int;
using GLsizei = int;
using GLfloat = float;
using GLdouble = double;
using GLchar = char;
using GLubyte = unsigned char;
using GLuint64 = std::uint64_t;
using GLsizeiptr = std::ptrdiff_t;
using GLintptr = std::ptrdiff_t;

// The constant names mirror the C API so code reads like any GL tutorial.
inline constexpr GLboolean GL_FALSE = 0;
inline constexpr GLboolean GL_TRUE = 1;
inline constexpr GLenum GL_NO_ERROR = 0;
inline constexpr GLenum GL_ZERO = 0;
inline constexpr GLenum GL_ONE = 1;

// Errors.
inline constexpr GLenum GL_INVALID_ENUM = 0x0500;
inline constexpr GLenum GL_INVALID_VALUE = 0x0501;
inline constexpr GLenum GL_INVALID_OPERATION = 0x0502;
inline constexpr GLenum GL_OUT_OF_MEMORY = 0x0505;
inline constexpr GLenum GL_INVALID_FRAMEBUFFER_OPERATION = 0x0506;

// Primitives.
inline constexpr GLenum GL_POINTS = 0x0000;
inline constexpr GLenum GL_LINES = 0x0001;
inline constexpr GLenum GL_LINE_STRIP = 0x0003;
inline constexpr GLenum GL_TRIANGLES = 0x0004;
inline constexpr GLenum GL_TRIANGLE_STRIP = 0x0005;

// Buffer bits.
inline constexpr GLbitfield GL_DEPTH_BUFFER_BIT = 0x00000100;
inline constexpr GLbitfield GL_STENCIL_BUFFER_BIT = 0x00000400;
inline constexpr GLbitfield GL_COLOR_BUFFER_BIT = 0x00004000;

// Depth functions.
inline constexpr GLenum GL_LESS = 0x0201;
inline constexpr GLenum GL_LEQUAL = 0x0203;
inline constexpr GLenum GL_ALWAYS = 0x0207;

// Blending.
inline constexpr GLenum GL_SRC_ALPHA = 0x0302;
inline constexpr GLenum GL_ONE_MINUS_SRC_ALPHA = 0x0303;

// Faces and polygon modes.
inline constexpr GLenum GL_FRONT = 0x0404;
inline constexpr GLenum GL_BACK = 0x0405;
inline constexpr GLenum GL_FRONT_AND_BACK = 0x0408;
inline constexpr GLenum GL_CW = 0x0900;
inline constexpr GLenum GL_CCW = 0x0901;
inline constexpr GLenum GL_LINE = 0x1B01;
inline constexpr GLenum GL_FILL = 0x1B02;

// Capabilities.
inline constexpr GLenum GL_LINE_SMOOTH = 0x0B20;
inline constexpr GLenum GL_CULL_FACE = 0x0B44;
inline constexpr GLenum GL_DEPTH_TEST = 0x0B71;
inline constexpr GLenum GL_BLEND = 0x0BE2;
inline constexpr GLenum GL_SCISSOR_TEST = 0x0C11;
inline constexpr GLenum GL_POLYGON_OFFSET_LINE = 0x2A02;
inline constexpr GLenum GL_POLYGON_OFFSET_FILL = 0x8037;
inline constexpr GLenum GL_MULTISAMPLE = 0x809D;
inline constexpr GLenum GL_FRAMEBUFFER_SRGB = 0x8DB9;

// Data types.
inline constexpr GLenum GL_UNSIGNED_BYTE = 0x1401;
inline constexpr GLenum GL_UNSIGNED_SHORT = 0x1403;
inline constexpr GLenum GL_INT = 0x1404;
inline constexpr GLenum GL_UNSIGNED_INT = 0x1405;
inline constexpr GLenum GL_FLOAT = 0x1406;

// Pixel formats.
inline constexpr GLenum GL_DEPTH_COMPONENT = 0x1902;
inline constexpr GLenum GL_RGB = 0x1907;
inline constexpr GLenum GL_RGBA = 0x1908;
inline constexpr GLenum GL_RGBA8 = 0x8058;
inline constexpr GLenum GL_DEPTH_COMPONENT24 = 0x81A6;
inline constexpr GLenum GL_DEPTH24_STENCIL8 = 0x88F0;

// Queries of implementation state.
inline constexpr GLenum GL_VIEWPORT = 0x0BA2;
inline constexpr GLenum GL_PACK_ALIGNMENT = 0x0D05;
inline constexpr GLenum GL_UNPACK_ALIGNMENT = 0x0CF5;
inline constexpr GLenum GL_VENDOR = 0x1F00;
inline constexpr GLenum GL_RENDERER = 0x1F01;
inline constexpr GLenum GL_VERSION = 0x1F02;
inline constexpr GLenum GL_SHADING_LANGUAGE_VERSION = 0x8B8C;
inline constexpr GLenum GL_MAX_SAMPLES = 0x8D57;

// Textures.
inline constexpr GLenum GL_TEXTURE_2D = 0x0DE1;
inline constexpr GLenum GL_TEXTURE0 = 0x84C0;
inline constexpr GLenum GL_TEXTURE_MAG_FILTER = 0x2800;
inline constexpr GLenum GL_TEXTURE_MIN_FILTER = 0x2801;
inline constexpr GLenum GL_TEXTURE_WRAP_S = 0x2802;
inline constexpr GLenum GL_TEXTURE_WRAP_T = 0x2803;
inline constexpr GLenum GL_NEAREST = 0x2600;
inline constexpr GLenum GL_LINEAR = 0x2601;
inline constexpr GLenum GL_CLAMP_TO_EDGE = 0x812F;

// Buffers.
inline constexpr GLenum GL_ARRAY_BUFFER = 0x8892;
inline constexpr GLenum GL_ELEMENT_ARRAY_BUFFER = 0x8893;
inline constexpr GLenum GL_STREAM_DRAW = 0x88E0;
inline constexpr GLenum GL_STATIC_DRAW = 0x88E4;
inline constexpr GLenum GL_DYNAMIC_DRAW = 0x88E8;

// Shaders.
inline constexpr GLenum GL_FRAGMENT_SHADER = 0x8B30;
inline constexpr GLenum GL_VERTEX_SHADER = 0x8B31;
inline constexpr GLenum GL_COMPILE_STATUS = 0x8B81;
inline constexpr GLenum GL_LINK_STATUS = 0x8B82;
inline constexpr GLenum GL_INFO_LOG_LENGTH = 0x8B84;

// Framebuffers.
inline constexpr GLenum GL_READ_FRAMEBUFFER = 0x8CA8;
inline constexpr GLenum GL_DRAW_FRAMEBUFFER = 0x8CA9;
inline constexpr GLenum GL_FRAMEBUFFER_COMPLETE = 0x8CD5;
inline constexpr GLenum GL_COLOR_ATTACHMENT0 = 0x8CE0;
inline constexpr GLenum GL_DEPTH_ATTACHMENT = 0x8D00;
inline constexpr GLenum GL_DEPTH_STENCIL_ATTACHMENT = 0x821A;
inline constexpr GLenum GL_FRAMEBUFFER = 0x8D40;
inline constexpr GLenum GL_RENDERBUFFER = 0x8D41;

// Timer queries (core since 3.3).
inline constexpr GLenum GL_QUERY_RESULT = 0x8866;
inline constexpr GLenum GL_QUERY_RESULT_AVAILABLE = 0x8867;
inline constexpr GLenum GL_TIME_ELAPSED = 0x88BF;

// X(return_type, name_without_gl_prefix, parameter_list)
#define AEROLAB_GL_FUNCTIONS(X)                                               \
  X(GLenum, GetError, (void))                                                 \
  X(const GLubyte*, GetString, (GLenum name))                                 \
  X(void, GetIntegerv, (GLenum pname, GLint * data))                          \
  X(void, Viewport, (GLint x, GLint y, GLsizei width, GLsizei height))        \
  X(void, Scissor, (GLint x, GLint y, GLsizei width, GLsizei height))         \
  X(void, Enable, (GLenum cap))                                               \
  X(void, Disable, (GLenum cap))                                              \
  X(void, Clear, (GLbitfield mask))                                           \
  X(void, ClearColor, (GLfloat r, GLfloat g, GLfloat b, GLfloat a))           \
  X(void, ClearDepth, (GLdouble depth))                                       \
  X(void, DepthFunc, (GLenum func))                                           \
  X(void, DepthMask, (GLboolean flag))                                        \
  X(void, ColorMask, (GLboolean r, GLboolean g, GLboolean b, GLboolean a))    \
  X(void, CullFace, (GLenum mode))                                            \
  X(void, FrontFace, (GLenum mode))                                           \
  X(void, BlendFunc, (GLenum sfactor, GLenum dfactor))                        \
  X(void, BlendFuncSeparate,                                                  \
    (GLenum src_rgb, GLenum dst_rgb, GLenum src_alpha, GLenum dst_alpha))     \
  X(void, PolygonMode, (GLenum face, GLenum mode))                            \
  X(void, PolygonOffset, (GLfloat factor, GLfloat units))                     \
  X(void, LineWidth, (GLfloat width))                                         \
  X(void, DrawArrays, (GLenum mode, GLint first, GLsizei count))              \
  X(void, DrawElements,                                                       \
    (GLenum mode, GLsizei count, GLenum type, const void* indices))           \
  X(void, ReadPixels,                                                         \
    (GLint x, GLint y, GLsizei width, GLsizei height, GLenum format,          \
     GLenum type, void* pixels))                                              \
  X(void, PixelStorei, (GLenum pname, GLint param))                           \
  X(void, Finish, (void))                                                     \
  X(void, GenTextures, (GLsizei n, GLuint * textures))                        \
  X(void, DeleteTextures, (GLsizei n, const GLuint* textures))                \
  X(void, BindTexture, (GLenum target, GLuint texture))                       \
  X(void, ActiveTexture, (GLenum texture))                                    \
  X(void, TexImage2D,                                                         \
    (GLenum target, GLint level, GLint internal_format, GLsizei width,        \
     GLsizei height, GLint border, GLenum format, GLenum type,                \
     const void* pixels))                                                     \
  X(void, TexParameteri, (GLenum target, GLenum pname, GLint param))          \
  X(void, GenBuffers, (GLsizei n, GLuint * buffers))                          \
  X(void, DeleteBuffers, (GLsizei n, const GLuint* buffers))                  \
  X(void, BindBuffer, (GLenum target, GLuint buffer))                         \
  X(void, BufferData,                                                         \
    (GLenum target, GLsizeiptr size, const void* data, GLenum usage))         \
  X(void, BufferSubData,                                                      \
    (GLenum target, GLintptr offset, GLsizeiptr size, const void* data))      \
  X(void, GenVertexArrays, (GLsizei n, GLuint * arrays))                      \
  X(void, DeleteVertexArrays, (GLsizei n, const GLuint* arrays))              \
  X(void, BindVertexArray, (GLuint array))                                    \
  X(void, EnableVertexAttribArray, (GLuint index))                            \
  X(void, VertexAttribPointer,                                                \
    (GLuint index, GLint size, GLenum type, GLboolean normalized,             \
     GLsizei stride, const void* pointer))                                    \
  X(GLuint, CreateShader, (GLenum type))                                      \
  X(void, ShaderSource,                                                       \
    (GLuint shader, GLsizei count, const GLchar* const* source,               \
     const GLint* length))                                                    \
  X(void, CompileShader, (GLuint shader))                                     \
  X(void, GetShaderiv, (GLuint shader, GLenum pname, GLint * params))         \
  X(void, GetShaderInfoLog,                                                   \
    (GLuint shader, GLsizei buffer_size, GLsizei * length, GLchar * log))     \
  X(void, DeleteShader, (GLuint shader))                                      \
  X(GLuint, CreateProgram, (void))                                            \
  X(void, AttachShader, (GLuint program, GLuint shader))                      \
  X(void, DetachShader, (GLuint program, GLuint shader))                      \
  X(void, LinkProgram, (GLuint program))                                      \
  X(void, GetProgramiv, (GLuint program, GLenum pname, GLint * params))       \
  X(void, GetProgramInfoLog,                                                  \
    (GLuint program, GLsizei buffer_size, GLsizei * length, GLchar * log))    \
  X(void, DeleteProgram, (GLuint program))                                    \
  X(void, UseProgram, (GLuint program))                                       \
  X(GLint, GetUniformLocation, (GLuint program, const GLchar* name))          \
  X(void, Uniform1i, (GLint location, GLint v0))                              \
  X(void, Uniform1f, (GLint location, GLfloat v0))                            \
  X(void, Uniform2f, (GLint location, GLfloat v0, GLfloat v1))                \
  X(void, Uniform3f, (GLint location, GLfloat v0, GLfloat v1, GLfloat v2))    \
  X(void, Uniform4f,                                                          \
    (GLint location, GLfloat v0, GLfloat v1, GLfloat v2, GLfloat v3))         \
  X(void, UniformMatrix3fv,                                                   \
    (GLint location, GLsizei count, GLboolean transpose,                      \
     const GLfloat* value))                                                   \
  X(void, UniformMatrix4fv,                                                   \
    (GLint location, GLsizei count, GLboolean transpose,                      \
     const GLfloat* value))                                                   \
  X(void, GenFramebuffers, (GLsizei n, GLuint * framebuffers))                \
  X(void, DeleteFramebuffers, (GLsizei n, const GLuint* framebuffers))        \
  X(void, BindFramebuffer, (GLenum target, GLuint framebuffer))               \
  X(void, FramebufferTexture2D,                                               \
    (GLenum target, GLenum attachment, GLenum texture_target, GLuint texture, \
     GLint level))                                                            \
  X(void, FramebufferRenderbuffer,                                            \
    (GLenum target, GLenum attachment, GLenum renderbuffer_target,            \
     GLuint renderbuffer))                                                    \
  X(GLenum, CheckFramebufferStatus, (GLenum target))                          \
  X(void, GenRenderbuffers, (GLsizei n, GLuint * renderbuffers))              \
  X(void, DeleteRenderbuffers, (GLsizei n, const GLuint* renderbuffers))      \
  X(void, BindRenderbuffer, (GLenum target, GLuint renderbuffer))             \
  X(void, RenderbufferStorageMultisample,                                     \
    (GLenum target, GLsizei samples, GLenum internal_format, GLsizei width,   \
     GLsizei height))                                                         \
  X(void, BlitFramebuffer,                                                    \
    (GLint src_x0, GLint src_y0, GLint src_x1, GLint src_y1, GLint dst_x0,    \
     GLint dst_y0, GLint dst_x1, GLint dst_y1, GLbitfield mask,               \
     GLenum filter))                                                          \
  X(void, GenQueries, (GLsizei n, GLuint * ids))                              \
  X(void, DeleteQueries, (GLsizei n, const GLuint* ids))                      \
  X(void, BeginQuery, (GLenum target, GLuint id))                             \
  X(void, EndQuery, (GLenum target))                                          \
  X(void, GetQueryObjectiv, (GLuint id, GLenum pname, GLint * params))        \
  X(void, GetQueryObjectui64v, (GLuint id, GLenum pname, GLuint64 * params))

#define AEROLAB_GL_DECLARE(return_type, name, params)            \
  using PFNGL_##name = return_type(AEROLAB_GL_APIENTRY*) params; \
  extern PFNGL_##name gl##name;
AEROLAB_GL_FUNCTIONS(AEROLAB_GL_DECLARE)
#undef AEROLAB_GL_DECLARE

// A generic function pointer, as returned by SDL_GL_GetProcAddress.
using GlProc = void (*)();
using GlProcLoader = GlProc (*)(const char* name);

// Resolves every function above through `loader`. Requires a current
// OpenGL 3.3 core context. Returns false and logs the names of any functions
// that could not be resolved.
bool LoadGlFunctions(GlProcLoader loader);

// Human-readable name for a glGetError() code.
std::string_view GlErrorName(GLenum error);

// Drains the GL error queue, logging each error with `context`. Returns true
// when no errors were pending.
bool CheckGlErrors(std::string_view context);

}  // namespace aerolab

#endif  // AEROLAB_RENDER_GL_H_
