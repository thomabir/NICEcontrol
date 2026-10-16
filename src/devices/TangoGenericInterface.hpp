#pragma once

#include <tango.h>

#include <memory>
#include <string>
#include <vector>

#include "data/Image.hpp"

class TangoGenericInterface {
 private:
  std::unique_ptr<Tango::DeviceProxy> tango_device;
  std::string device_name;
  bool connected = false;

 public:
  TangoGenericInterface(const std::string &name = "motor/shutter/1") : device_name(name) {}

  bool is_connected() { return connected; }

  int create_device_proxy();
  int ping_device();
  int initialise_if_not_running();
  int disconnect();
  int connect();

  // Run a command without argument. Give back the error description, or an empty string on success.
  std::string run_command(const std::string &command);

  bool has_command(const std::string &command);

  // The first description of a Tango error, which is the description from the device.
  static std::string error_description(const Tango::DevFailed &e);

  // A scalar attribute as the device reports it.
  struct ScalarAttribute {
    std::string name;
    int data_type = 0;  // a Tango::CmdArgType
    bool writable = false;
    std::string unit;
    std::vector<std::string> enum_labels;
  };

  // All scalar attributes with a type that extract_scalar and write_scalar_text can convert.
  std::vector<ScalarAttribute> get_scalar_attributes();

  static bool is_supported_scalar_type(int data_type);

  // Start an asynchronous read of several attributes. Give back the request id, or -1 on failure.
  long start_read(const std::vector<std::string> &names);

  enum class ReadReply { Pending, Arrived, Failed };

  // Look for the reply of an asynchronous read without a wait.
  ReadReply poll_read(long request_id, std::vector<Tango::DeviceAttribute> &values);

  // Convert a read scalar value. Numbers, booleans and enum indices go to number. Strings and states go to text.
  // Give back false if the read of this attribute failed.
  static bool extract_scalar(Tango::DeviceAttribute &att, int data_type, double &number, std::string &text);

  // Write a scalar attribute from its value as text. Give back the error description, or an empty string on success.
  std::string write_scalar_text(const std::string &attribute_name, int data_type, const std::string &text);

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

  // Run a command and give back its reply. Ret is the type that the Tango extraction operator gives.
  template <typename Ret, typename Arg>
  Ret run_command_with_reply(const std::string &command, const Arg &argument) {
    Tango::DeviceData device_data;
    Arg argument_copy = argument;
    device_data << argument_copy;
    std::string command_copy = command;

    try {
      Tango::DeviceData reply = tango_device->command_inout(command_copy, device_data);
      Ret value;
      reply >> value;
      return value;
    } catch (Tango::DevFailed &e) {
      Tango::Except::print_exception(e);
      return Ret();
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

  // Read a spectrum attribute and give back only its read values.
  // A READ_WRITE spectrum carries the read values and then the set values in one sequence.
  template <typename T>
  std::vector<T> read_spectrum_attribute(const std::string &attribute_name) {
    try {
      std::string attribute_name_copy = attribute_name;
      Tango::DeviceAttribute att_reply = tango_device->read_attribute(attribute_name_copy);
      const size_t n_read = static_cast<size_t>(att_reply.get_nb_read());
      std::vector<T> value;
      att_reply >> value;
      if (n_read < value.size()) {
        value.resize(n_read);
      }
      return value;
    } catch (Tango::DevFailed &e) {
      Tango::Except::print_exception(e);
      return std::vector<T>();
    }
  }

  // Write a spectrum attribute. T is the exact Tango type, for example Tango::DevULong.
  template <typename T>
  void write_spectrum_attribute(const std::string &attribute_name, const std::vector<T> &value) {
    try {
      std::string attribute_name_copy = attribute_name;
      std::vector<T> value_copy = value;
      Tango::DeviceAttribute att(attribute_name_copy, value_copy);
      tango_device->write_attribute(att);
    } catch (Tango::DevFailed &e) {
      Tango::Except::print_exception(e);
    }
  }

  // strings need special treatment with tango: const strings are not allowed, so we need to copy them
  void write_attribute(const std::string &attribute_name, const std::string &value);

  std::vector<std::string> get_commands();
};
