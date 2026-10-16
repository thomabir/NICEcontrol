#include "devices/TangoGenericInterface.hpp"

#include <algorithm>

namespace {

template <typename T>
bool extract_number(Tango::DeviceAttribute &att, double &number) {
  T value;
  if (!(att >> value)) {
    return false;
  }
  number = static_cast<double>(value);
  return true;
}

}  // namespace

int TangoGenericInterface::create_device_proxy() {
  // If there is already a device proxy, no need to create a new one
  if (tango_device != nullptr) {
    return 0;
  }

  // Create a new device proxy
  try {
    tango_device = std::make_unique<Tango::DeviceProxy>(device_name.c_str());
    return 0;
  } catch (Tango::DevFailed &e) {
    Tango::Except::print_exception(e);
    return -1;
  }
}

int TangoGenericInterface::ping_device() {
  try {
    tango_device->ping();
    return 0;
  } catch (Tango::DevFailed &e) {
    Tango::Except::print_exception(e);
    return -1;
  }
}

int TangoGenericInterface::initialise_if_not_running() {
  try {
    Tango::DevState state;
    Tango::DeviceAttribute att_reply;

    att_reply = tango_device->read_attribute("State");
    att_reply >> state;

    if (state != Tango::ON && has_command("Initialise")) {
      run_command("Initialise");
    }
    return 0;  // success
  } catch (Tango::DevFailed &e) {
    Tango::Except::print_exception(e);
    return -1;  // failure
  }
}

int TangoGenericInterface::disconnect() {
  tango_device.reset();
  connected = false;
  return 0;
}

int TangoGenericInterface::connect() {
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

std::string TangoGenericInterface::run_command(const std::string &command) {
  // tango expects a non-const string
  std::string command_copy = command;
  try {
    tango_device->command_inout(command_copy);
    return "";
  } catch (Tango::DevFailed &e) {
    Tango::Except::print_exception(e);
    return error_description(e);
  }
}

bool TangoGenericInterface::has_command(const std::string &command) {
  const std::vector<std::string> commands = get_commands();
  return std::find(commands.begin(), commands.end(), command) != commands.end();
}

std::string TangoGenericInterface::error_description(const Tango::DevFailed &e) {
  if (e.errors.length() == 0) {
    return "unknown Tango error";
  }
  return std::string(e.errors[0].desc.in());
}

std::vector<TangoGenericInterface::ScalarAttribute> TangoGenericInterface::get_scalar_attributes() {
  std::vector<ScalarAttribute> attributes;
  try {
    std::unique_ptr<Tango::AttributeInfoListEx> list(tango_device->attribute_list_query_ex());
    for (const Tango::AttributeInfoEx &info : *list) {
      if (info.data_format != Tango::SCALAR || !is_supported_scalar_type(info.data_type)) {
        continue;
      }
      ScalarAttribute attribute;
      attribute.name = info.name;
      attribute.data_type = info.data_type;
      attribute.writable = info.writable == Tango::READ_WRITE || info.writable == Tango::WRITE;
      attribute.unit = (info.unit == "No unit") ? "" : info.unit;
      attribute.enum_labels = info.enum_labels;
      attributes.push_back(attribute);
    }
  } catch (Tango::DevFailed &e) {
    Tango::Except::print_exception(e);
  }
  return attributes;
}

bool TangoGenericInterface::is_supported_scalar_type(int data_type) {
  switch (data_type) {
    case Tango::DEV_BOOLEAN:
    case Tango::DEV_SHORT:
    case Tango::DEV_USHORT:
    case Tango::DEV_LONG:
    case Tango::DEV_ULONG:
    case Tango::DEV_LONG64:
    case Tango::DEV_ULONG64:
    case Tango::DEV_FLOAT:
    case Tango::DEV_DOUBLE:
    case Tango::DEV_UCHAR:
    case Tango::DEV_ENUM:
    case Tango::DEV_STRING:
    case Tango::DEV_STATE:
      return true;
    default:
      return false;
  }
}

long TangoGenericInterface::start_read(const std::vector<std::string> &names) {
  try {
    std::vector<std::string> names_copy = names;
    return tango_device->read_attributes_asynch(names_copy);
  } catch (Tango::DevFailed &e) {
    Tango::Except::print_exception(e);
    return -1;
  }
}

TangoGenericInterface::ReadReply TangoGenericInterface::poll_read(long request_id,
                                                                  std::vector<Tango::DeviceAttribute> &values) {
  try {
    std::unique_ptr<std::vector<Tango::DeviceAttribute>> reply(tango_device->read_attributes_reply(request_id));
    values = std::move(*reply);
    return ReadReply::Arrived;
  } catch (Tango::AsynReplyNotArrived &) {
    return ReadReply::Pending;
  } catch (Tango::DevFailed &e) {
    Tango::Except::print_exception(e);
    return ReadReply::Failed;
  }
}

bool TangoGenericInterface::extract_scalar(Tango::DeviceAttribute &att, int data_type, double &number,
                                           std::string &text) {
  try {
    if (att.has_failed()) {
      return false;
    }
    switch (data_type) {
      case Tango::DEV_BOOLEAN:
        return extract_number<Tango::DevBoolean>(att, number);
      case Tango::DEV_SHORT:
      case Tango::DEV_ENUM:
        return extract_number<Tango::DevShort>(att, number);
      case Tango::DEV_USHORT:
        return extract_number<Tango::DevUShort>(att, number);
      case Tango::DEV_LONG:
        return extract_number<Tango::DevLong>(att, number);
      case Tango::DEV_ULONG:
        return extract_number<Tango::DevULong>(att, number);
      case Tango::DEV_LONG64:
        return extract_number<Tango::DevLong64>(att, number);
      case Tango::DEV_ULONG64:
        return extract_number<Tango::DevULong64>(att, number);
      case Tango::DEV_FLOAT:
        return extract_number<Tango::DevFloat>(att, number);
      case Tango::DEV_DOUBLE:
        return extract_number<Tango::DevDouble>(att, number);
      case Tango::DEV_UCHAR:
        return extract_number<Tango::DevUChar>(att, number);
      case Tango::DEV_STRING:
        return static_cast<bool>(att >> text);
      case Tango::DEV_STATE: {
        Tango::DevState state;
        if (!(att >> state)) {
          return false;
        }
        text = Tango::DevStateName[state];
        return true;
      }
      default:
        return false;
    }
  } catch (Tango::DevFailed &) {
    return false;
  }
}

std::string TangoGenericInterface::write_scalar_text(const std::string &attribute_name, int data_type,
                                                     const std::string &text) {
  try {
    std::string name = attribute_name;
    Tango::DeviceAttribute att;
    switch (data_type) {
      case Tango::DEV_BOOLEAN:
        att = Tango::DeviceAttribute(name, static_cast<bool>(std::stod(text) != 0.0));
        break;
      case Tango::DEV_SHORT:
      case Tango::DEV_ENUM:
        att = Tango::DeviceAttribute(name, static_cast<Tango::DevShort>(std::stol(text)));
        break;
      case Tango::DEV_USHORT:
        att = Tango::DeviceAttribute(name, static_cast<Tango::DevUShort>(std::stoul(text)));
        break;
      case Tango::DEV_LONG:
        att = Tango::DeviceAttribute(name, static_cast<Tango::DevLong>(std::stol(text)));
        break;
      case Tango::DEV_ULONG:
        att = Tango::DeviceAttribute(name, static_cast<Tango::DevULong>(std::stoul(text)));
        break;
      case Tango::DEV_LONG64:
        att = Tango::DeviceAttribute(name, static_cast<Tango::DevLong64>(std::stoll(text)));
        break;
      case Tango::DEV_ULONG64:
        att = Tango::DeviceAttribute(name, static_cast<Tango::DevULong64>(std::stoull(text)));
        break;
      case Tango::DEV_FLOAT:
        att = Tango::DeviceAttribute(name, std::stof(text));
        break;
      case Tango::DEV_DOUBLE:
        att = Tango::DeviceAttribute(name, std::stod(text));
        break;
      case Tango::DEV_UCHAR:
        att = Tango::DeviceAttribute(name, static_cast<Tango::DevUChar>(std::stoul(text)));
        break;
      case Tango::DEV_STRING: {
        std::string value = text;
        att = Tango::DeviceAttribute(name, value);
        break;
      }
      default:
        return "the type of attribute " + attribute_name + " is not writable from text";
    }
    tango_device->write_attribute(att);
    return "";
  } catch (Tango::DevFailed &e) {
    Tango::Except::print_exception(e);
    return error_description(e);
  } catch (const std::exception &) {
    return "\"" + text + "\" is not a valid value for attribute " + attribute_name;
  }
}

void TangoGenericInterface::write_attribute(const std::string &attribute_name, const std::string &value) {
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

std::vector<std::string> TangoGenericInterface::get_commands() {
  std::vector<std::string> commands;

  // connect
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
