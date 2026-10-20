#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>

namespace stab {

// Blocking single-lock queue with a capacity, used for back-pressure between
// pipeline stages.
template <class T>
class BoundedQueue {
 public:
  explicit BoundedQueue(size_t capacity) : capacity_(capacity) {}

  // Blocks while full. Returns false if the queue was closed.
  bool push(T value) {
    std::unique_lock lock(mutex_);
    notFull_.wait(lock, [this] { return closed_ || items_.size() < capacity_; });
    if (closed_) return false;
    items_.push_back(std::move(value));
    notEmpty_.notify_one();
    return true;
  }

  // Blocks while empty. Returns nullopt once closed and drained.
  std::optional<T> pop() {
    std::unique_lock lock(mutex_);
    notEmpty_.wait(lock, [this] { return closed_ || !items_.empty(); });
    if (items_.empty()) return std::nullopt;
    T value = std::move(items_.front());
    items_.pop_front();
    notFull_.notify_one();
    return value;
  }

  // No more pushes; consumers drain what is left.
  void close() {
    std::lock_guard lock(mutex_);
    closed_ = true;
    notFull_.notify_all();
    notEmpty_.notify_all();
  }

  // Like close(), but also discards queued items.
  void cancel() {
    std::lock_guard lock(mutex_);
    closed_ = true;
    items_.clear();
    notFull_.notify_all();
    notEmpty_.notify_all();
  }

 private:
  std::mutex mutex_;
  std::condition_variable notFull_;
  std::condition_variable notEmpty_;
  std::deque<T> items_;
  size_t capacity_;
  bool closed_ = false;
};

}  // namespace stab
