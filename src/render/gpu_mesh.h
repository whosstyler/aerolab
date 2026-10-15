#ifndef AEROLAB_RENDER_GPU_MESH_H_
#define AEROLAB_RENDER_GPU_MESH_H_

#include <span>

#include "geometry/debug_draw.h"
#include "geometry/mesh_data.h"
#include "math/aabb.h"
#include "render/gl.h"

namespace aerolab {

// An indexed triangle mesh resident on the GPU (VAO + vertex and index
// buffers). Move-only; GL objects are released with it.
class GpuMesh {
 public:
  static GpuMesh Upload(const MeshData& mesh);

  GpuMesh() = default;
  ~GpuMesh();
  GpuMesh(GpuMesh&& other) noexcept;
  GpuMesh& operator=(GpuMesh&& other) noexcept;
  GpuMesh(const GpuMesh&) = delete;
  GpuMesh& operator=(const GpuMesh&) = delete;

  void Draw() const;

  GLsizei index_count() const { return index_count_; }
  const Aabb& bounds() const { return bounds_; }

 private:
  void Release();

  GLuint vao_ = 0;
  GLuint vertex_buffer_ = 0;
  GLuint index_buffer_ = 0;
  GLsizei index_count_ = 0;
  Aabb bounds_;
};

// Streams debug line vertices to the GPU each frame and draws them as
// GL_LINES. Move-only.
class LineBatch {
 public:
  static LineBatch Create();

  LineBatch() = default;
  ~LineBatch();
  LineBatch(LineBatch&& other) noexcept;
  LineBatch& operator=(LineBatch&& other) noexcept;
  LineBatch(const LineBatch&) = delete;
  LineBatch& operator=(const LineBatch&) = delete;

  // Uploads `vertices` (pairs of segment endpoints) and draws them.
  void Draw(std::span<const DebugVertex> vertices);

 private:
  void Release();

  GLuint vao_ = 0;
  GLuint vertex_buffer_ = 0;
};

// Vertex array with no buffers, for draws whose vertices are generated in
// the vertex shader from gl_VertexID (fullscreen passes).
class EmptyVertexArray {
 public:
  static EmptyVertexArray Create();

  EmptyVertexArray() = default;
  ~EmptyVertexArray();
  EmptyVertexArray(EmptyVertexArray&& other) noexcept;
  EmptyVertexArray& operator=(EmptyVertexArray&& other) noexcept;
  EmptyVertexArray(const EmptyVertexArray&) = delete;
  EmptyVertexArray& operator=(const EmptyVertexArray&) = delete;

  void Bind() const { glBindVertexArray(vao_); }

 private:
  GLuint vao_ = 0;
};

}  // namespace aerolab

#endif  // AEROLAB_RENDER_GPU_MESH_H_
