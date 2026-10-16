#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <stop_token>
#include <thread>

#include "core/Commands.hpp"
#include "core/Whiteboard.hpp"

// The control core. It runs on one thread at a fixed cycle period and it is independent of the user interface.
//
// Each cycle runs three steps in order:
//   sense  every application reads its hardware and writes what it found on the whiteboard
//   plan   every application decides what it wants, and writes that on the whiteboard
//   act    every application sends its commands to its own hardware
// Only the act step reaches an actuator, so each actuator takes at most one command per cycle.
//
// The core steers the modes and leaves the details to the applications.
// The applications are defined in Core.cpp only, thus a file that includes this header does not include the device
// libraries.
class Core {
 public:
  static constexpr std::chrono::milliseconds kCyclePeriod{10};

  Core();
  ~Core();

  void start();
  void request_stop();

  Whiteboard &whiteboard() { return wb; }
  CommandBox &commands() { return box; }

 private:
  struct Apps;

  Whiteboard wb;
  CommandBox box;

  // core.time_s counts from this point.
  const int64_t t_PC_start = wb.clocks.t_PC_now();

  std::unique_ptr<Apps> apps;

  std::jthread thread;

  void run(std::stop_token stop);
  void cycle();
};
