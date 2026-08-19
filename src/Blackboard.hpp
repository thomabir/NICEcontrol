#pragma once

#include <mutex>
#include <utility>

#include "Image.hpp"

// The blackboard is the private data of the core. No application depends on it. It carries what only the log and the
// user interface need, and what is too large for the whiteboard snapshot.

// Holds the newest value. One writer, any number of readers.
template <typename T>
class Latest {
 public:
  void store(T new_value) {
    std::lock_guard<std::mutex> lock(mutex);
    value = std::move(new_value);
  }

  T load() {
    std::lock_guard<std::mutex> lock(mutex);
    return value;
  }

 private:
  T value;
  std::mutex mutex;
};

struct Blackboard {
  Latest<Image<int>> camera_image;
};
