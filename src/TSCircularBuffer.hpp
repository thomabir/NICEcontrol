#pragma once

#include <boost/circular_buffer.hpp>
#include <mutex>

// thread-safe circular buffer
template <typename T>
class TSCircularBuffer {
 private:
  boost::circular_buffer<T> m_buffer;
  std::mutex m_mutex;

 public:
  TSCircularBuffer() : m_buffer(1000000) {}       // default size: 1e6
  TSCircularBuffer(int size) : m_buffer(size) {}  // custom size

  void push(T item) {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_buffer.push_back(item);
  }

  T front() {
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_buffer.front();
  }

  T back() {
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_buffer.back();
  }

  T pop() {
    std::unique_lock<std::mutex> lock(m_mutex);
    T item = m_buffer.front();
    m_buffer.pop_front();
    return item;
  }

  bool isempty() {
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_buffer.empty();
  }

  int size() {
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_buffer.size();
  }
};