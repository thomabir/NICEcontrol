#pragma once

#include <string>
#include <vector>

// How the user interface shows and edits the value of a Tango attribute.
enum class TangoValueKind { Number, Integer, Boolean, Enum, Text };

// One scalar attribute of a Tango device, with its last read value.
struct TangoAttributeState {
  std::string name;
  TangoValueKind kind = TangoValueKind::Number;
  int data_type = 0;  // the Tango type, for the conversion of a written value
  bool writable = false;
  std::string unit;
  std::vector<std::string> enum_labels;
  bool valid = false;   // the last read gave a value
  double number = 0.0;  // the value of the kinds Number, Integer, Boolean and Enum
  std::string text;     // the value of the kind Text
};
