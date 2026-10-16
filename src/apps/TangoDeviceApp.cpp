#include "apps/TangoDeviceApp.hpp"

#include <memory>

#include "devices/TangoGenericInterface.hpp"

TangoDeviceApp::TangoDeviceApp(const std::string &device_name, TangoDeviceState &device_state)
    : device(std::make_unique<TangoGenericInterface>(device_name)), state(device_state) {}

TangoDeviceApp::~TangoDeviceApp() = default;

void TangoDeviceApp::sense() {
  if (!state.connected) {
    connect();
    return;
  }
  read_attributes();
}

void TangoDeviceApp::act(const TangoDeviceCommands &command) {
  if (!state.connected) {
    return;
  }
  if (seen_valid && command.device_command_count != seen_count) {
    state.last_error = device->run_command(command.device_command);
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

void TangoDeviceApp::connect() {
  // A connection attempt is slow when the device server is absent, so it happens once per retry interval.
  if (--retry_countdown > 0) {
    return;
  }
  retry_countdown = kRetryCycles;

  if (device->connect() < 0) {
    return;
  }
  state.device_commands = device->get_commands();
  state.attributes.clear();
  attribute_names.clear();
  for (const TangoGenericInterface::ScalarAttribute &attribute : device->get_scalar_attributes()) {
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

TangoValueKind TangoDeviceApp::kind_of(int data_type) {
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

void TangoDeviceApp::read_attributes() {
  if (attribute_names.empty()) {
    return;
  }
  if (read_request >= 0) {
    std::vector<Tango::DeviceAttribute> values;
    const TangoGenericInterface::ReadReply reply = device->poll_read(read_request, values);
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
  read_request = device->start_read(attribute_names);
  read_countdown = kReadCycles;
}

void TangoDeviceApp::write_attribute(const std::string &name, const std::string &value) {
  for (const TangoAttributeState &a : state.attributes) {
    if (a.name == name) {
      state.last_error = device->write_scalar_text(name, a.data_type, value);
      return;
    }
  }
  state.last_error = "the device has no attribute " + name;
}
