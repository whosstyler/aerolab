#ifndef AEROLAB_CORE_RING_BUFFER_H_
#define AEROLAB_CORE_RING_BUFFER_H_

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <utility>
#include <vector>

namespace aerolab {

// Fixed-capacity FIFO that overwrites its oldest element when full. Used for
// motion trails, telemetry histories and frame-time graphs.
//
// Index 0 is the oldest element and size() - 1 the newest.
template <typename T>
class RingBuffer {
 public:
  explicit RingBuffer(std::size_t capacity = 0)
      : storage_(std::max<std::size_t>(capacity, 1)),
        capacity_(std::max<std::size_t>(capacity, 1)) {}

  void Push(const T& value) {
    storage_[(head_ + size_) % capacity_] = value;
    if (size_ < capacity_) {
      ++size_;
    } else {
      head_ = (head_ + 1) % capacity_;
    }
  }

  void Clear() {
    head_ = 0;
    size_ = 0;
  }

  // Changes the capacity, keeping the newest elements that still fit.
  void SetCapacity(std::size_t capacity) {
    capacity = std::max<std::size_t>(capacity, 1);
    if (capacity == capacity_) return;
    std::vector<T> kept;
    const std::size_t keep = std::min(size_, capacity);
    kept.reserve(capacity);
    for (std::size_t i = size_ - keep; i < size_; ++i)
      kept.push_back((*this)[i]);
    kept.resize(capacity);
    storage_ = std::move(kept);
    capacity_ = capacity;
    head_ = 0;
    size_ = keep;
  }

  const T& operator[](std::size_t i) const {
    assert(i < size_);
    return storage_[(head_ + i) % capacity_];
  }

  const T& front() const { return (*this)[0]; }
  const T& back() const { return (*this)[size_ - 1]; }

  std::size_t size() const { return size_; }
  std::size_t capacity() const { return capacity_; }
  bool empty() const { return size_ == 0; }
  bool full() const { return size_ == capacity_; }

  // Copies the contents, oldest first, into a contiguous vector.
  std::vector<T> ToVector() const {
    std::vector<T> out;
    out.reserve(size_);
    for (std::size_t i = 0; i < size_; ++i) out.push_back((*this)[i]);
    return out;
  }

 private:
  std::vector<T> storage_;
  std::size_t capacity_;
  std::size_t head_ = 0;
  std::size_t size_ = 0;
};

}  // namespace aerolab

#endif  // AEROLAB_CORE_RING_BUFFER_H_
