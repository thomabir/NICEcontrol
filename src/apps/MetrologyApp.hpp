#pragma once

#include <cstddef>

#include "core/Whiteboard.hpp"

// Reads the metrology ADC stream, which arrives over UDP. The socket does not block, so the core drains whatever has
// arrived and moves on.
class MetrologyApp {
 public:
  explicit MetrologyApp(Whiteboard &whiteboard);
  ~MetrologyApp();

  void sense();

 private:
  static constexpr int kPortNumber = 12345;
  static constexpr int kChannelsPerRecord = 17;  // one counter and sixteen channels
  static constexpr int kTimepointsPerPacket = 10;
  static constexpr size_t kPacketBytes = sizeof(int) * kChannelsPerRecord * kTimepointsPerPacket;
  static constexpr int kMaxPacketsPerCycle = 200;

  Whiteboard &wb;
  int sockfd = -1;

  void open_socket();
};
