#pragma once

#include <string>

#include "../Commands.hpp"
#include "../TangoGenericInterface.hpp"
#include "../Whiteboard.hpp"

// A Tango device that takes commands and holds no state of its own, such as the shutter or the neutral density
// filter. The application finds the command list once and sends one command when the counter changes.
class TangoDeviceApp {
 public:
  TangoDeviceApp(const std::string &device_name, TangoDeviceState &state) : device(device_name), state(state) {}

  void sense() {
    if (state.connected) {
      return;
    }
    // A connection attempt is slow when the device server is absent, so it happens once per retry interval.
    if (--retry_countdown > 0) {
      return;
    }
    retry_countdown = kRetryCycles;

    if (device.connect() < 0) {
      return;
    }
    state.device_commands = device.get_commands();
    state.connected = true;
  }

  void act(const TangoDeviceCommands &command) {
    if (!state.connected) {
      return;
    }
    if (seen_valid && command.device_command_count != seen_count) {
      device.run_command(command.device_command);
    }
    seen_count = command.device_command_count;
    seen_valid = true;
  }

 private:
  static constexpr int kRetryCycles = 500;

  TangoGenericInterface device;
  TangoDeviceState &state;
  int retry_countdown = 1;
  uint32_t seen_count = 0;
  bool seen_valid = false;
};
