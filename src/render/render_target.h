#ifndef AEROLAB_RENDER_RENDER_TARGET_H_
#define AEROLAB_RENDER_RENDER_TARGET_H_

#include "render/gl.h"

namespace aerolab {

// Off-screen colour + depth target for the 3D viewport.
//
// The scene is drawn into a (possibly multisampled) framebuffer and then
// resolved into a regular texture. The texture object keeps the same GL name
// across resizes, so UI code may hold on to it.
class RenderTarget {
 public:
  RenderTarget() = default;
  ~RenderTarget();
  RenderTarget(const RenderTarget&) = delete;
  RenderTarget& operator=(const RenderTarget&) = delete;

  // (Re)allocates storage when the size or sample count changes. `samples`
  // is clamped to what the driver supports; 0 or 1 disables MSAA. Returns
  // false (after logging) if the framebuffer is incomplete.
  bool Resize(int width, int height, int samples);

  // Binds the drawing framebuffer and sets the viewport to cover it.
  void BindForDrawing() const;

  // Resolves multisampled colour into color_texture().
  void Resolve() const;

  // Copies the resolved image into the window's back buffer, scaled to the
  // given size. Leaves framebuffer 0 bound.
  void BlitToWindow(int window_width, int window_height) const;

  GLuint color_texture() const { return resolve_texture_; }
  GLuint resolve_framebuffer() const { return resolve_framebuffer_; }
  int width() const { return width_; }
  int height() const { return height_; }
  int samples() const { return samples_; }

 private:
  void CreateObjects();
  // Allocates storage for the given size; returns false if incomplete.
  bool Allocate(int width, int height, int samples);

  GLuint draw_framebuffer_ = 0;
  GLuint color_renderbuffer_ = 0;
  GLuint depth_renderbuffer_ = 0;
  GLuint resolve_framebuffer_ = 0;
  GLuint resolve_texture_ = 0;
  int width_ = 0;
  int height_ = 0;
  int samples_ = -1;
};

}  // namespace aerolab

#endif  // AEROLAB_RENDER_RENDER_TARGET_H_
