#pragma once

#include <tango.h>

#include <array>
#include <cmath>
#include <ctime>

class TangoGenericInterface {
 private:
  Tango::DeviceProxy *tango_device;
  bool initialized;
  std::string device_name;

 public:
  TangoGenericInterface(const std::string &device_name = "detectors/flir/1")
      : tango_device(nullptr), initialized(false), device_name(device_name) {
    init();
  }

  ~TangoGenericInterface() {
    if (tango_device) {
      delete tango_device;
    }
  }

  bool is_initialized() { return initialized; }

  void init() {
    // Check if device is already initialized
    if (tango_device == nullptr) {
      try {
        tango_device = new Tango::DeviceProxy(device_name.c_str());
      } catch (Tango::DevFailed &e) {
        Tango::Except::print_exception(e);
        return;
      }
    }

    // Get the state and check if it's running
    try {
      tango_device->ping();
      Tango::DevState state;
      Tango::DeviceAttribute att_reply;

      att_reply = tango_device->read_attribute("State");
      att_reply >> state;

      if (state != 1) {
        std::cout << "Device is already running" << std::endl;
        initialized = true;
        return;
      }

      // Use a mutable string for the command
      std::string init_command = "Initialise";
      tango_device->command_inout(init_command);
      initialized = true;
    } catch (Tango::DevFailed &e) {
      Tango::Except::print_exception(e);
      return;
    }
  }

  void run_command(std::string &command) {
    if (!initialized) {
      throw std::runtime_error("Device not initialized. Call init() first.");
    }

    Tango::DeviceAttribute att_reply;
    Tango::DevState state;

    try {
      tango_device->ping();
      att_reply = tango_device->read_attribute("State");
      att_reply >> state;

      if (state == 1) {
        throw std::invalid_argument("Device is off");
      }

      tango_device->command_inout(command);
    } catch (Tango::DevFailed &e) {
      Tango::Except::print_exception(e);
      return;
    }
  }

  void run_command(const std::string &command) {
    // Create a copy internally to pass to the non-const version
    std::string command_copy = command;
    run_command(command_copy);
  }

  std::vector<std::string> get_commands() {
    std::vector<std::string> commands;
    try {
      auto reply = tango_device->command_list_query();
      for (const auto &cmd_info : reply[0]) {
        if (cmd_info.cmd_name != "Init") {
          commands.push_back(cmd_info.cmd_name);
        }
      }
    } catch (Tango::DevFailed &e) {
      Tango::Except::print_exception(e);
    }
    return commands;
  }
};
