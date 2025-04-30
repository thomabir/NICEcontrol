/**
 * @file SPMCRingBuffer.hpp
 * @brief Single-producer multi-consumer ring buffer implementation
 */

#include <array>
#include <memory>
#include <vector>

#include "Consumer.hpp"

/**
 * @class SPMCRingBuffer
 * @brief A thread-safe single-producer multi-consumer ring buffer
 *
 * This template class implements a ring buffer that supports multiple consumers
 * reading from the same buffer. Pushing to the buffer is wait-free. Each
 * consumer maintains its own read position.
 *
 * @tparam T The type of elements stored in the buffer
 * @tparam Capacity The maximum number of elements the buffer can hold
 */
template <typename T, size_t Capacity>
class SPMCRingBuffer {
 public:
  /**
   * @brief Constructs an empty ring buffer
   */
  SPMCRingBuffer() : buffer(), writeIndex(0), numConsumers(0) {}

  /**
   * @brief Creates a new consumer for this buffer
   * @return Pointer to the newly created consumer
   */
  Consumer* subscribe() {
    // Create a new consumer, starting at the current write index
    auto consumer = std::make_unique<Consumer>(writeIndex);

    // Get a pointer to the consumer
    Consumer* conPtr = consumer.get();

    // Add the consumer to the list of consumers
    consumers.push_back(std::move(consumer));
    numConsumers++;

    return conPtr;
  }

  /**
   * @brief Adds an item to the buffer and notifies all consumers
   * @param item The item to add to the buffer
   */
  void push(const T& item) {
    buffer[writeIndex] = item;  // Add the item to the buffer
    advanceIndex(writeIndex);   // Advance the write index

    // Notify all consumers
    for (size_t i = 0; i < numConsumers; ++i) {
      consumers[i]->notify();
    }
  }

  /**
   * @brief Retrieves an item from the buffer, blocking until data is available
   * @param consumer Pointer to the consumer requesting data
   * @param item Reference to store the retrieved item
   */
  T pop(Consumer* consumer) {
    consumer->wait();                      // Wait if there is no data available
    T item = buffer[consumer->readIndex];  // Read the item
    advanceIndex(consumer->readIndex);     // Advance the read index
    return item;
  }

  /**
   * @brief Attempts to retrieve an item from the buffer without blocking
   * @param consumer Pointer to the consumer requesting data
   * @param item Reference to store the retrieved item
   * @return true if an item was retrieved, false if no data was available
   */
  bool try_pop(Consumer* consumer, T& item) {
    if (consumer->try_wait()) {            // If new data is available
      item = buffer[consumer->readIndex];  // Read the item
      advanceIndex(consumer->readIndex);   // Advance the read index
      return true;
    }
    return false;
  }

 private:
  std::array<T, Capacity> buffer;                    ///< Storage for buffer elements
  std::vector<std::unique_ptr<Consumer>> consumers;  ///< List of consumers
  size_t writeIndex;                                 ///< Current write position
  size_t numConsumers;                               ///< Number of consumers subscribed

  /**
   * @brief Advances an index with wraparound
   * @param index The index to advance
   */
  void advanceIndex(size_t& index) { index = (index + 1) % Capacity; }
};
