#include "render/gpu_mesh.h"

#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>

namespace aerolab {
namespace {

// Byte offset into the bound vertex buffer, in the form GL expects.
const void* BufferOffset(std::size_t bytes) {
  // GL's vertex-attribute API encodes buffer offsets as pointers.
  return reinterpret_cast<const void*>(  // NOLINT(performance-no-int-to-ptr)
      static_cast<std::uintptr_t>(bytes));
}

GLsizeiptr ByteSize(std::size_t count, std::size_t element_size) {
  return static_cast<GLsizeiptr>(count * element_size);
}

}  // namespace

GpuMesh GpuMesh::Upload(const MeshData& mesh) {
  GpuMesh result;
  result.bounds_ = mesh.bounds;
  result.index_count_ = static_cast<GLsizei>(mesh.indices.size());

  glGenVertexArrays(1, &result.vao_);
  glGenBuffers(1, &result.vertex_buffer_);
  glGenBuffers(1, &result.index_buffer_);

  glBindVertexArray(result.vao_);
  glBindBuffer(GL_ARRAY_BUFFER, result.vertex_buffer_);
  glBufferData(GL_ARRAY_BUFFER,
               ByteSize(mesh.vertices.size(), sizeof(MeshVertex)),
               mesh.vertices.data(), GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, result.index_buffer_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,
               ByteSize(mesh.indices.size(), sizeof(std::uint32_t)),
               mesh.indices.data(), GL_STATIC_DRAW);

  constexpr GLsizei kStride = sizeof(MeshVertex);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, kStride,
                        BufferOffset(offsetof(MeshVertex, px)));
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, kStride,
                        BufferOffset(offsetof(MeshVertex, nx)));
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, kStride,
                        BufferOffset(offsetof(MeshVertex, r)));
  glBindVertexArray(0);
  return result;
}

GpuMesh::~GpuMesh() { Release(); }

GpuMesh::GpuMesh(GpuMesh&& other) noexcept
    : vao_(std::exchange(other.vao_, 0)),
      vertex_buffer_(std::exchange(other.vertex_buffer_, 0)),
      index_buffer_(std::exchange(other.index_buffer_, 0)),
      index_count_(std::exchange(other.index_count_, 0)),
      bounds_(other.bounds_) {}

GpuMesh& GpuMesh::operator=(GpuMesh&& other) noexcept {
  if (this != &other) {
    Release();
    vao_ = std::exchange(other.vao_, 0);
    vertex_buffer_ = std::exchange(other.vertex_buffer_, 0);
    index_buffer_ = std::exchange(other.index_buffer_, 0);
    index_count_ = std::exchange(other.index_count_, 0);
    bounds_ = other.bounds_;
  }
  return *this;
}

void GpuMesh::Release() {
  if (vao_ != 0) glDeleteVertexArrays(1, &vao_);
  if (vertex_buffer_ != 0) glDeleteBuffers(1, &vertex_buffer_);
  if (index_buffer_ != 0) glDeleteBuffers(1, &index_buffer_);
  vao_ = vertex_buffer_ = index_buffer_ = 0;
}

void GpuMesh::Draw() const {
  if (index_count_ == 0) return;
  glBindVertexArray(vao_);
  glDrawElements(GL_TRIANGLES, index_count_, GL_UNSIGNED_INT, nullptr);
}

LineBatch LineBatch::Create() {
  LineBatch batch;
  glGenVertexArrays(1, &batch.vao_);
  glGenBuffers(1, &batch.vertex_buffer_);
  glBindVertexArray(batch.vao_);
  glBindBuffer(GL_ARRAY_BUFFER, batch.vertex_buffer_);
  constexpr GLsizei kStride = sizeof(DebugVertex);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, kStride,
                        BufferOffset(offsetof(DebugVertex, x)));
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, kStride,
                        BufferOffset(offsetof(DebugVertex, color)));
  glBindVertexArray(0);
  return batch;
}

LineBatch::~LineBatch() { Release(); }

LineBatch::LineBatch(LineBatch&& other) noexcept
    : vao_(std::exchange(other.vao_, 0)),
      vertex_buffer_(std::exchange(other.vertex_buffer_, 0)) {}

LineBatch& LineBatch::operator=(LineBatch&& other) noexcept {
  if (this != &other) {
    Release();
    vao_ = std::exchange(other.vao_, 0);
    vertex_buffer_ = std::exchange(other.vertex_buffer_, 0);
  }
  return *this;
}

void LineBatch::Release() {
  if (vao_ != 0) glDeleteVertexArrays(1, &vao_);
  if (vertex_buffer_ != 0) glDeleteBuffers(1, &vertex_buffer_);
  vao_ = vertex_buffer_ = 0;
}

void LineBatch::Draw(std::span<const DebugVertex> vertices) {
  if (vertices.size() < 2) return;
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
  // Re-specifying the whole store each frame lets the driver orphan the old
  // buffer instead of stalling on in-flight draws.
  glBufferData(GL_ARRAY_BUFFER, ByteSize(vertices.size(), sizeof(DebugVertex)),
               vertices.data(), GL_STREAM_DRAW);
  glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(vertices.size()));
}

EmptyVertexArray EmptyVertexArray::Create() {
  EmptyVertexArray result;
  glGenVertexArrays(1, &result.vao_);
  return result;
}

EmptyVertexArray::~EmptyVertexArray() {
  if (vao_ != 0) glDeleteVertexArrays(1, &vao_);
}

EmptyVertexArray::EmptyVertexArray(EmptyVertexArray&& other) noexcept
    : vao_(std::exchange(other.vao_, 0)) {}

EmptyVertexArray& EmptyVertexArray::operator=(
    EmptyVertexArray&& other) noexcept {
  if (this != &other) {
    if (vao_ != 0) glDeleteVertexArrays(1, &vao_);
    vao_ = std::exchange(other.vao_, 0);
  }
  return *this;
}

}  // namespace aerolab
