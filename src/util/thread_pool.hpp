#pragma once

#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <type_traits>
#include <vector>

namespace stab {

// Fixed-size FIFO thread pool. Tasks still queued at destruction are dropped
// (their futures report broken_promise).
class ThreadPool {
 public:
  explicit ThreadPool(unsigned threads) {
    for (unsigned i = 0; i < threads; ++i) workers_.emplace_back([this] { work(); });
  }

  ~ThreadPool() {
    {
      std::lock_guard lock(mutex_);
      stopping_ = true;
      tasks_ = {};
    }
    cv_.notify_all();
    for (auto& w : workers_) w.join();
  }

  ThreadPool(const ThreadPool&) = delete;
  ThreadPool& operator=(const ThreadPool&) = delete;

  template <class F>
  std::future<std::invoke_result_t<F>> submit(F&& f) {
    using R = std::invoke_result_t<F>;
    auto task = std::make_shared<std::packaged_task<R()>>(std::forward<F>(f));
    auto future = task->get_future();
    {
      std::lock_guard lock(mutex_);
      tasks_.emplace([task] { (*task)(); });
    }
    cv_.notify_one();
    return future;
  }

 private:
  void work() {
    for (;;) {
      std::function<void()> task;
      {
        std::unique_lock lock(mutex_);
        cv_.wait(lock, [this] { return stopping_ || !tasks_.empty(); });
        if (stopping_) return;
        task = std::move(tasks_.front());
        tasks_.pop();
      }
      task();
    }
  }

  std::mutex mutex_;
  std::condition_variable cv_;
  std::queue<std::function<void()>> tasks_;
  std::vector<std::thread> workers_;
  bool stopping_ = false;
};

}  // namespace stab
