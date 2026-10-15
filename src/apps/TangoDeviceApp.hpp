#pragma once

#include <string>
#include <vector>

#include "core/Commands.hpp"
#include "core/Whiteboard.hpp"
#include "devices/TangoGenericInterface.hpp"

// A Tango device that the user interface controls through its commands and its scalar attributes, such as the
// shutter or the chopper. The application finds the commands and the attributes once at the connection.
// It reads all scalar attributes at an interval with an asynchronous read, thus the cycle does not wait for the device.
// It sends one command or one attribute write when the counter of that request changes.
class TangoDeviceApp {
 public:
  TangoDeviceApp(const std::string &device_name, TangoDeviceState &device_state)
      : device(device_name), state(device_state) {}

  void sense() {
    if (!state.connected) {
      connect();
      return;
    }
    read_attributes();
  }

  void act(const TangoDeviceCommands &command) {
    if (!state.connected) {
      return;
    }
    if (seen_valid && command.device_command_count != seen_count) {
      state.last_error = device.run_command(command.device_command);
      read_countdown = 1;
    }
    if (seen_valid && command.write_count != seen_write_count) {
      write_attribute(command.write_attribute, command.write_value);
      read_countdown = 1;
    }
    seen_count = command.device_command_count;
    seen_write_count = command.write_count;
    seen_valid = true;
  }

 private:
  static constexpr int kRetryCycles = 500;
  static constexpr int kReadCycles = 50;

  TangoGenericInterface device;
  TangoDeviceState &state;
  std::vector<std::string> attribute_names;
  int retry_countdown = 1;
  int read_countdown = 1;
  long read_request = -1;
  uint32_t seen_count = 0;
  uint32_t seen_write_count = 0;
  bool seen_valid = false;

  void connect() {
    // A connection attempt is slow when the device server is absent, so it happens once per retry interval.
    if (--retry_countdown > 0) {
      return;
    }
    retry_countdown = kRetryCycles;

    if (device.connect() < 0) {
      return;
    }
    state.device_commands = device.get_commands();
    state.attributes.clear();
    attribute_names.clear();
    for (const TangoGenericInterface::ScalarAttribute &attribute : device.get_scalar_attributes()) {
      TangoAttributeState a;
      a.name = attribute.name;
      a.kind = kind_of(attribute.data_type);
      a.data_type = attribute.data_type;
      a.writable = attribute.writable;
      a.unit = attribute.unit;
      a.enum_labels = attribute.enum_labels;
      state.attributes.push_back(a);
      attribute_names.push_back(attribute.name);
    }
    state.connected = true;
  }

  static TangoValueKind kind_of(int data_type) {
    switch (data_type) {
      case Tango::DEV_FLOAT:
      case Tango::DEV_DOUBLE:
        return TangoValueKind::Number;
      case Tango::DEV_BOOLEAN:
        return TangoValueKind::Boolean;
      case Tango::DEV_ENUM:
        return TangoValueKind::Enum;
      case Tango::DEV_STRING:
      case Tango::DEV_STATE:
        return TangoValueKind::Text;
      default:
        return TangoValueKind::Integer;
    }
  }

  void read_attributes() {
    if (attribute_names.empty()) {
      return;
    }
    if (read_request >= 0) {
      std::vector<Tango::DeviceAttribute> values;
      const TangoGenericInterface::ReadReply reply = device.poll_read(read_request, values);
      if (reply == TangoGenericInterface::ReadReply::Pending) {
        return;
      }
      read_request = -1;
      for (size_t i = 0; i < state.attributes.size(); i++) {
        TangoAttributeState &a = state.attributes[i];
        a.valid = reply == TangoGenericInterface::ReadReply::Arrived && i < values.size() &&
                  TangoGenericInterface::extract_scalar(values[i], a.data_type, a.number, a.text);
      }
      return;
    }
    if (--read_countdown > 0) {
      return;
    }
    read_request = device.start_read(attribute_names);
    read_countdown = kReadCycles;
  }

  void write_attribute(const std::string &name, const std::string &value) {
    for (const TangoAttributeState &a : state.attributes) {
      if (a.name == name) {
        state.last_error = device.write_scalar_text(name, a.data_type, value);
        return;
      }
    }
    state.last_error = "the device has no attribute " + name;
  }
};
