#include "render/render_target.h"

#include <algorithm>

#include "core/log.h"

namespace aerolab {

RenderTarget::~RenderTarget() {
  if (draw_framebuffer_ != 0) glDeleteFramebuffers(1, &draw_framebuffer_);
  if (resolve_framebuffer_ != 0) {
    glDeleteFramebuffers(1, &resolve_framebuffer_);
  }
  if (color_renderbuffer_ != 0) {
    glDeleteRenderbuffers(1, &color_renderbuffer_);
  }
  if (depth_renderbuffer_ != 0) {
    glDeleteRenderbuffers(1, &depth_renderbuffer_);
  }
  if (resolve_texture_ != 0) glDeleteTextures(1, &resolve_texture_);
}

void RenderTarget::CreateObjects() {
  glGenFramebuffers(1, &draw_framebuffer_);
  glGenFramebuffers(1, &resolve_framebuffer_);
  glGenRenderbuffers(1, &color_renderbuffer_);
  glGenRenderbuffers(1, &depth_renderbuffer_);
  glGenTextures(1, &resolve_texture_);
}

bool RenderTarget::Resize(int width, int height, int samples) {
  width = std::max(width, 1);
  height = std::max(height, 1);
  GLint max_samples = 0;
  glGetIntegerv(GL_MAX_SAMPLES, &max_samples);
  samples = std::clamp(samples, 0, std::max(max_samples, 0));
  if (samples == 1) samples = 0;
  if (width == width_ && height == height_ && samples == samples_) return true;
  if (draw_framebuffer_ == 0) CreateObjects();

  if (Allocate(width, height, samples)) return true;
  // Some drivers reject particular MSAA formats; fall back to no MSAA.
  if (samples > 0 && Allocate(width, height, 0)) {
    LogWarning("{}x MSAA unavailable; rendering without multisampling",
               samples);
    return true;
  }
  // Forget the cached size so the next call retries instead of reusing an
  // incomplete framebuffer.
  width_ = height_ = 0;
  samples_ = -1;
  return false;
}

bool RenderTarget::Allocate(int width, int height, int samples) {
  width_ = width;
  height_ = height;
  samples_ = samples;

  // Multisampled colour and depth for drawing. samples == 0 allocates
  // ordinary single-sample storage through the same entry point.
  glBindRenderbuffer(GL_RENDERBUFFER, color_renderbuffer_);
  glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_RGBA8, width,
                                   height);
  glBindRenderbuffer(GL_RENDERBUFFER, depth_renderbuffer_);
  glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples,
                                   GL_DEPTH24_STENCIL8, width, height);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);

  glBindFramebuffer(GL_FRAMEBUFFER, draw_framebuffer_);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                            GL_RENDERBUFFER, color_renderbuffer_);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT,
                            GL_RENDERBUFFER, depth_renderbuffer_);
  const GLenum draw_status = glCheckFramebufferStatus(GL_FRAMEBUFFER);

  // Single-sample texture that receives the resolved image.
  glBindTexture(GL_TEXTURE_2D, resolve_texture_);
  glTexImage2D(GL_TEXTURE_2D, 0, static_cast<GLint>(GL_RGBA8), width, height, 0,
               GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                  static_cast<GLint>(GL_LINEAR));
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
                  static_cast<GLint>(GL_LINEAR));
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,
                  static_cast<GLint>(GL_CLAMP_TO_EDGE));
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,
                  static_cast<GLint>(GL_CLAMP_TO_EDGE));
  glBindTexture(GL_TEXTURE_2D, 0);

  glBindFramebuffer(GL_FRAMEBUFFER, resolve_framebuffer_);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                         resolve_texture_, 0);
  const GLenum resolve_status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  if (draw_status != GL_FRAMEBUFFER_COMPLETE ||
      resolve_status != GL_FRAMEBUFFER_COMPLETE) {
    LogError("Viewport framebuffer incomplete (draw 0x{:X}, resolve 0x{:X})",
             draw_status, resolve_status);
    return false;
  }
  return CheckGlErrors("RenderTarget::Resize");
}

void RenderTarget::BindForDrawing() const {
  glBindFramebuffer(GL_FRAMEBUFFER, draw_framebuffer_);
  glViewport(0, 0, width_, height_);
}

void RenderTarget::Resolve() const {
  glBindFramebuffer(GL_READ_FRAMEBUFFER, draw_framebuffer_);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, resolve_framebuffer_);
  glBlitFramebuffer(0, 0, width_, height_, 0, 0, width_, height_,
                    GL_COLOR_BUFFER_BIT, GL_NEAREST);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void RenderTarget::BlitToWindow(int window_width, int window_height) const {
  glBindFramebuffer(GL_READ_FRAMEBUFFER, resolve_framebuffer_);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
  glBlitFramebuffer(0, 0, width_, height_, 0, 0, window_width, window_height,
                    GL_COLOR_BUFFER_BIT, GL_LINEAR);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

}  // namespace aerolab
