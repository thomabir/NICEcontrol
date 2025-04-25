/*
Example of a client using the TANGO C++ api. Runs an intensity calibration routine
*/

#pragma once

#include <tango.h>

#include <array>
#include <cmath>
#include <ctime>
#include <numeric>
#include <vector>

class CameraInterface {
 private:
  Tango::DeviceProxy *cam_device;
  bool initialized;

 public:
  CameraInterface() : cam_device(nullptr), initialized(false) {
    // run init function
    init();
  }

  ~CameraInterface() {
    if (cam_device) {
      delete cam_device;
    }
  }

  bool is_initialized() { return initialized; }

  void init() {
    // Check if camera is already initialized
    if (cam_device == nullptr) {
      try {
        cam_device = new Tango::DeviceProxy("detectors/flir/1");
      } catch (Tango::DevFailed &e) {
        Tango::Except::print_exception(e);
        return;
      }
    }

    // Get the state and check if it's running
    try {
      cam_device->ping();
      Tango::DevState state;
      Tango::DeviceAttribute att_reply;

      att_reply = cam_device->read_attribute("State");
      att_reply >> state;

      if (state != 1) {
        std::cout << "Camera is already running" << std::endl;
        initialized = true;
        return;
      }

      // Use a mutable string for the command
      std::string init_command = "Initialise";
      cam_device->command_inout(init_command);
      initialized = true;
    } catch (Tango::DevFailed &e) {
      Tango::Except::print_exception(e);
      return;
    }
  }

  void run_command(std::string &command) {
    if (!initialized) {
      throw std::runtime_error("Camera not initialized. Call init() first.");
    }

    Tango::DeviceAttribute att_reply;
    Tango::DevState state;

    try {
      cam_device->ping();
      att_reply = cam_device->read_attribute("State");
      att_reply >> state;

      if (state == 1) {
        throw std::invalid_argument("Camera off");
      }

      cam_device->command_inout(command);
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

  void start_stream() {
    std::string command = "StartStream";
    run_command(command);
  }

  void stop_stream() {
    std::string command = "StopFrames";
    run_command(command);
  }

  void set_height(ulong height) {
    if (!initialized) {
      throw std::runtime_error("Camera not initialized. Call init() first.");
    }

    Tango::DeviceAttribute att_reply;
    Tango::DevState state;

    try {
      cam_device->ping();
      att_reply = cam_device->read_attribute("State");
      att_reply >> state;

      if (state == 1) {
        throw std::invalid_argument("Camera off");
      }

      Tango::DeviceAttribute height_att("Height", 0);
      height_att << height;
      cam_device->write_attribute(height_att);
    } catch (Tango::DevFailed &e) {
      Tango::Except::print_exception(e);
      return;
    }
  }

  std::vector<std::string> get_commands() {
    std::vector<std::string> commands;
    try {
      auto reply = cam_device->command_list_query();
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

  int get_width() {
    if (!initialized) {
      throw std::runtime_error("Camera not initialized. Call init() first.");
    }

    Tango::DeviceAttribute att_reply;
    Tango::DevState state;

    try {
      cam_device->ping();
      att_reply = cam_device->read_attribute("State");
      att_reply >> state;

      if (state == 1) {
        throw std::invalid_argument("Camera off");
      }

      att_reply = cam_device->read_attribute("Image");
      return att_reply.get_dim_x();
    } catch (Tango::DevFailed &e) {
      Tango::Except::print_exception(e);
      return -1;
    }
  }

  int get_height() {
    if (!initialized) {
      throw std::runtime_error("Camera not initialized. Call init() first.");
    }

    Tango::DeviceAttribute att_reply;
    Tango::DevState state;

    try {
      cam_device->ping();
      att_reply = cam_device->read_attribute("State");
      att_reply >> state;

      if (state == 1) {
        throw std::invalid_argument("Camera off");
      }

      att_reply = cam_device->read_attribute("Image");
      return att_reply.get_dim_y();
    } catch (Tango::DevFailed &e) {
      Tango::Except::print_exception(e);
      return -1;
    }
  }

  std::vector<unsigned short> get_image() {
    if (!initialized) {
      throw std::runtime_error("Camera not initialized. Call init() first.");
    }

    Tango::DeviceAttribute att_reply;
    Tango::DevState state;

    try {
      cam_device->ping();
      att_reply = cam_device->read_attribute("State");
      att_reply >> state;

      if (state == 1) {
        throw std::invalid_argument("Camera off");
      }

      att_reply = cam_device->read_attribute("Image");
      std::vector<unsigned short> image;
      att_reply >> image;

      return image;
    } catch (Tango::DevFailed &e) {
      Tango::Except::print_exception(e);
      return std::vector<unsigned short>();
    }
  }
};
