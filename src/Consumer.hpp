/**
 * @file Consumer.hpp
 * @brief Class for managing consumers in a ring buffer implementation
 */
#pragma once
#include <semaphore>

/**
 * @class Consumer
 * @brief Manages read operations for a consumer of a multi-consumer ring buffer
 *
 * This class tracks the read position of a consumer and provides
 * synchronization primitives to notify consumers when data is available for
 * consumption.
 */
class Consumer {
 public:
  size_t readIndex;  ///< The index the consumer is currently reading from

  /**
   * @brief Constructs a Consumer with a specified read position
   * @param readIndex Initial read position for this consumer
   */
  Consumer(size_t readIndex) : readIndex(readIndex), sem(0) {}

  /**
   * @brief Notifies the consumer that there is data available
   */
  void notify() { sem.release(); }

  /**
   * @brief Blocks until data is available
   */
  void wait() { sem.acquire(); }

  /**
   * @brief Non-blocking check for data availability
   * @return true if data is available, false otherwise
   */
  bool try_wait() { return sem.try_acquire(); }

 private:
  std::counting_semaphore<1000000> sem;  ///< Number of items available to consume
};
