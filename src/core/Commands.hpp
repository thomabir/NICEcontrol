#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "data/PhotometryRegions.hpp"

// Commands travel from the outside world to the core. The core reads them once per cycle and acts on them in the act
// step. A command that holds a value takes effect when the value differs from the one the core last sent to the
// hardware. A command that triggers an action carries a counter, and the core acts when the counter changes.

struct OpdCommands {
  int mode = 0;  // 0 off, 1 open loop, 3 closed loop
  float setpoint_um = 0.0f;
  float open_loop_cmd_um = 0.0f;
  float kp = 0.0f;
  float ki = 1.0f;
  uint32_t reset_unwrap_count = 0;
};

struct TipTiltCommands {
  int mode = 0;  // 0 open loop, 1 closed loop, 2 hold still
  float x1 = 0.0f;
  float y1 = 0.0f;
  float x2 = 0.0f;
  float y2 = 0.0f;
};

struct CameraCommands {
  bool connect = false;

  // The camera reports its settings, and the core seeds these fields from that report. Until then the core sends
  // nothing, so that the program can start without a user interface.
  bool settings_valid = false;
  double framerate = 0.0;
  double integration_time_ms = 0.0;
  unsigned int width = 0;
  unsigned int height = 0;
  std::string filename;
  bool subtract_background = false;
  std::vector<PhotRegion> regions;

  // 0 takes the raw image, 1 takes the image with the background subtracted.
  int image_product = 0;

  unsigned int record_frames = 1;
  uint32_t record_count = 0;
  unsigned int background_frames = 1;
  uint32_t background_count = 0;

  std::string device_command;
  uint32_t device_command_count = 0;
};

struct TangoDeviceCommands {
  std::string device_command;
  uint32_t device_command_count = 0;
};

struct Commands {
  OpdCommands opd;
  TipTiltCommands tiptilt;
  CameraCommands camera;
  TangoDeviceCommands shutter;
  TangoDeviceCommands ndfilter;
};

// The one place where commands are stored. Later this becomes a scheduler that merges several command sources.
class CommandBox {
 public:
  Commands get() {
    std::lock_guard<std::mutex> lock(mutex);
    return commands;
  }

  template <typename F>
  void edit(F change) {
    std::lock_guard<std::mutex> lock(mutex);
    change(commands);
  }

 private:
  Commands commands;
  std::mutex mutex;
};
