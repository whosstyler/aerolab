#include "render/gl.h"

#include <string_view>

#include "core/log.h"

namespace aerolab {

#define AEROLAB_GL_DEFINE(return_type, name, params) \
  PFNGL_##name gl##name = nullptr;
AEROLAB_GL_FUNCTIONS(AEROLAB_GL_DEFINE)
#undef AEROLAB_GL_DEFINE

namespace {

void ResetGlFunctions() {
#define AEROLAB_GL_RESET(return_type, name, params) gl##name = nullptr;
  AEROLAB_GL_FUNCTIONS(AEROLAB_GL_RESET)
#undef AEROLAB_GL_RESET
}

}  // namespace

bool LoadGlFunctions(GlProcLoader loader) {
  if (loader == nullptr) return false;
  int missing = 0;
#define AEROLAB_GL_LOAD(return_type, name, params)               \
  gl##name = reinterpret_cast<PFNGL_##name>(loader("gl" #name)); \
  if (gl##name == nullptr) {                                     \
    LogError("OpenGL function gl{} is unavailable", #name);      \
    ++missing;                                                   \
  }
  AEROLAB_GL_FUNCTIONS(AEROLAB_GL_LOAD)
#undef AEROLAB_GL_LOAD
  // All-or-nothing: never leave a partially loaded function table behind.
  if (missing > 0) ResetGlFunctions();
  return missing == 0;
}

std::string_view GlErrorName(GLenum error) {
  switch (error) {
    case GL_NO_ERROR:
      return "GL_NO_ERROR";
    case GL_INVALID_ENUM:
      return "GL_INVALID_ENUM";
    case GL_INVALID_VALUE:
      return "GL_INVALID_VALUE";
    case GL_INVALID_OPERATION:
      return "GL_INVALID_OPERATION";
    case GL_OUT_OF_MEMORY:
      return "GL_OUT_OF_MEMORY";
    case GL_INVALID_FRAMEBUFFER_OPERATION:
      return "GL_INVALID_FRAMEBUFFER_OPERATION";
    default:
      return "unknown GL error";
  }
}

bool CheckGlErrors(std::string_view context) {
  bool clean = true;
  // Bounded so a lost context (which can report errors forever) cannot hang.
  for (int i = 0; i < 16; ++i) {
    const GLenum error = glGetError();
    if (error == GL_NO_ERROR) break;
    clean = false;
    LogError("OpenGL error {} (0x{:04X}) at {}", GlErrorName(error), error,
             context);
  }
  return clean;
}

}  // namespace aerolab
