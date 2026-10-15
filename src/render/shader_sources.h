#ifndef AEROLAB_RENDER_SHADER_SOURCES_H_
#define AEROLAB_RENDER_SHADER_SOURCES_H_

#include <string_view>

namespace aerolab::shaders {

// GLSL 3.30 core sources, embedded so the application has no runtime assets.
// Colour uniforms are sRGB; lighting happens in linear space.

extern const std::string_view kSkyVertex;
extern const std::string_view kSkyFragment;

extern const std::string_view kGroundVertex;
extern const std::string_view kGroundFragment;

extern const std::string_view kMeshVertex;
extern const std::string_view kMeshFragment;

extern const std::string_view kLineVertex;
extern const std::string_view kLineFragment;

extern const std::string_view kShadowVertex;
extern const std::string_view kShadowFragment;

}  // namespace aerolab::shaders

#endif  // AEROLAB_RENDER_SHADER_SOURCES_H_
