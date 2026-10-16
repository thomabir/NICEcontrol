#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "core/Commands.hpp"
#include "core/Whiteboard.hpp"

class TangoGenericInterface;

// A Tango device that the user interface controls through its commands and its scalar attributes, such as the
// shutter or the chopper. The application finds the commands and the attributes once at the connection.
// It reads all scalar attributes at an interval with an asynchronous read, thus the cycle does not wait for the device.
// It sends one command or one attribute write when the counter of that request changes.
class TangoDeviceApp {
 public:
  TangoDeviceApp(const std::string &device_name, TangoDeviceState &device_state);
  ~TangoDeviceApp();

  void sense();
  void act(const TangoDeviceCommands &command);

 private:
  static constexpr int kRetryCycles = 500;
  static constexpr int kReadCycles = 50;

  std::unique_ptr<TangoGenericInterface> device;
  TangoDeviceState &state;
  std::vector<std::string> attribute_names;
  int retry_countdown = 1;
  int read_countdown = 1;
  long read_request = -1;
  uint32_t seen_count = 0;
  uint32_t seen_write_count = 0;
  bool seen_valid = false;

  void connect();
  static TangoValueKind kind_of(int data_type);
  void read_attributes();
  void write_attribute(const std::string &name, const std::string &value);
};
