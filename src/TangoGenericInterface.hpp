#pragma once

#include <tango.h>

#include <array>
#include <cmath>
#include <ctime>
#include <vector>

#include "Image.hpp"

class TangoGenericInterface {
 private:
  Tango::DeviceProxy *tango_device;
  std::string device_name;
  bool connected = false;
  Tango::DeviceProxy *get_device() { return tango_device; }

 public:
  TangoGenericInterface(const std::string &device_name = "detectors/flir/1")
      : tango_device(nullptr), device_name(device_name) {}

  ~TangoGenericInterface() {
    if (tango_device) {
      delete tango_device;
    }
  }

  bool is_connected() { return connected; }

  int create_device_proxy() {
    // If there is already a device proxy, no need to create a new one
    if (tango_device != nullptr) {
      return 0;
    }

    // Create a new device proxy
    try {
      tango_device = new Tango::DeviceProxy(device_name.c_str());
      return 0;
    } catch (Tango::DevFailed &e) {
      Tango::Except::print_exception(e);
      tango_device = nullptr;
      return -1;
    }
  }

  int ping_device() {
    try {
      // tango_device->ping();  // TODO check if something comes back
      std::cout << " device ping took " << tango_device->ping() << " microseconds" << endl;
      return 0;
    } catch (Tango::DevFailed &e) {
      Tango::Except::print_exception(e);
      return -1;
    }
  }

  int initialise_if_not_running() {
    try {
      Tango::DevState state;
      Tango::DeviceAttribute att_reply;

      att_reply = tango_device->read_attribute("State");
      att_reply >> state;

      if (state != Tango::ON) {
        run_command("Initialise");
      }
      return 0;  // success
    } catch (Tango::DevFailed &e) {
      Tango::Except::print_exception(e);
      return -1;  // failure
    }
  }

  int disconnect() {
    return -1;  // not implemented
  }

  int connect() {
    // Create device proxy
    if (create_device_proxy() < 0) {
      return -1;
    }

    // Ping the device to check if it is reachable
    if (ping_device() < 0) {
      return -1;
    }

    // Initialise the device if it is not already running
    if (initialise_if_not_running() < 0) {
      return -1;
    }

    connected = true;
    return 0;
  }

  void run_command(const std::string &command) {
    // tango expects a non-const string
    std::string command_copy = command;
    try {
      tango_device->command_inout(command_copy);
    } catch (Tango::DevFailed &e) {
      Tango::Except::print_exception(e);
      return;
    }
  }

  template <typename T>
  void run_command(const std::string &command, T &argument) {
    Tango::DeviceData device_data;
    device_data << argument;
    std::string command_copy = command;

    try {
      tango_device->command_inout(command_copy, device_data);
    } catch (Tango::DevFailed &e) {
      Tango::Except::print_exception(e);
      return;
    }
  }

  template <typename T>
  T read_attribute(const std::string &attribute_name) {
    try {
      Tango::DeviceAttribute att_reply;
      std::string attribute_name_copy = attribute_name;
      att_reply = tango_device->read_attribute(attribute_name_copy);
      T value;
      att_reply >> value;
      return value;
    } catch (Tango::DevFailed &e) {
      Tango::Except::print_exception(e);
      return T();
    }
  }

  template <typename T>
  Image<T> read_image(const std::string &attribute_name) {
    try {
      Tango::DeviceAttribute att_reply;
      std::string attribute_name_copy = attribute_name;
      att_reply = tango_device->read_attribute(attribute_name_copy);
      std::vector<T> value;
      att_reply >> value;
      Image<T> image;
      image.data = value;
      image.width = att_reply.get_dim_x();
      image.height = att_reply.get_dim_y();
      return image;
    } catch (Tango::DevFailed &e) {
      Tango::Except::print_exception(e);
      return Image<T>();
    }
  }

  template <typename T>
  void write_attribute(const std::string &attribute_name, const T &value) {
    try {
      std::string attribute_name_copy = attribute_name;
      Tango::DeviceAttribute att(attribute_name_copy, T());
      att << value;
      tango_device->write_attribute(att);
    } catch (Tango::DevFailed &e) {
      Tango::Except::print_exception(e);
    }
  }

  // strings need special treatment with tango: const strings are not allowed, so we need to copy them
  void write_attribute(const std::string &attribute_name, const std::string &value) {
    try {
      std::string attribute_name_copy = attribute_name;
      Tango::DeviceAttribute att(attribute_name_copy, "");
      std::string value_copy = value;
      att << value_copy;
      tango_device->write_attribute(att);
    } catch (Tango::DevFailed &e) {
      Tango::Except::print_exception(e);
    }
  }

  std::vector<std::string> get_commands() {
    std::vector<std::string> commands;
    try {
      auto reply = tango_device->command_list_query();  // FIXME: Segfaults if not connected to a device
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

  std::vector<std::string> get_attributes() {
    std::vector<std::string> attributes;
    try {
      auto reply = tango_device->attribute_list_query();
      for (const auto &att_info : reply[0]) {
        attributes.push_back(att_info.name);
        std::cout << att_info.name << " " << att_info.data_type << " " << att_info.data_format << " "
                  << att_info.writable << " " << att_info.unit << " " << att_info.max_dim_x << " " << att_info.max_dim_y
                  << " " << att_info.max_value << " " << att_info.min_value << " ";
        // print the attribute value using get_att_details
        // get_att_details(att_info.name, att_info.data_type);
      }

    } catch (Tango::DevFailed &e) {
      Tango::Except::print_exception(e);
    }
    return attributes;
  }

  // void get_att_details(const std::string &attribute_name, int data_type) {
  //   Tango::DeviceAttribute att_reply;
  //   std::string att_name = attribute_name;
  //   att_reply = cam_device->read_attribute(att_name);
  //   if (data_type == Tango::DEV_STRING) {
  //     std::string value;
  //     att_reply >> value;
  //     std::cout << "Value: " << value << std::endl;
  //   } else if (data_type == Tango::DEV_DOUBLE) {
  //     double value;
  //     att_reply >> value;
  //     std::cout << "Value: " << value << std::endl;
  //   } else if (data_type == Tango::DEV_LONG) {
  //     int value;
  //     att_reply >> value;
  //     std::cout << "Value: " << value << std::endl;
  //   } else if (data_type == Tango::DEV_SHORT) {
  //     short value;
  //     att_reply >> value;
  //     std::cout << "Value: " << value << std::endl;
  //   } else if (data_type == Tango::DEV_ULONG) {
  //     unsigned int value;
  //     att_reply >> value;
  //     std::cout << "Value: " << value << std::endl;
  //   } else if (data_type == Tango::DEV_USHORT) {
  //     unsigned short value;
  //     att_reply >> value;
  //     std::cout << "Value: " << value << std::endl;
  //   } else if (data_type == Tango::DEV_BOOLEAN) {
  //     bool value;
  //     att_reply >> value;
  //     std::cout << "Value: " << value << std::endl;
  //   }
  // }

  //   std::vector<std::string> get_commands() {
  //     std::vector<std::string> commands;
  //     try {
  //       auto reply = tango_device->command_list_query();
  //       for (const auto &cmd_info : reply[0]) {
  //         if (cmd_info.cmd_name != "Init") {
  //           commands.push_back(cmd_info.cmd_name);
  //           std::cout << cmd_info.cmd_name << " "
  //           << cmd_info.cmd_tag << " "
  //           << cmd_info.in_type << " "
  //           << cmd_info.in_type_desc << " "
  //           << cmd_info.out_type << " "
  //           << cmd_info.out_type_desc << " "
  //           << std::endl;
  //         }
  //       }
  //     } catch (Tango::DevFailed &e) {
  //       Tango::Except::print_exception(e);
  //     }
  //     return commands;
  //   }
};
