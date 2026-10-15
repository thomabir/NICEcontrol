#pragma once

#include <array>
#include <cfloat>
#include <cstdio>
#include <map>
#include <string>

#include "data/TangoAttribute.hpp"
#include "imgui.h"

// The edit buffers of the attribute inputs, by "<window id>/<attribute name>".
// A buffer follows the read value while its input is not in use.
struct TangoEditBuffers {
  std::map<std::string, double> numbers;
  std::map<std::string, std::array<char, 256>> texts;
  std::map<std::string, bool> active;
};

inline std::string TangoValueText(const TangoAttributeState &a) {
  char buffer[64];
  switch (a.kind) {
    case TangoValueKind::Number:
      std::snprintf(buffer, sizeof(buffer), "%.6g", a.number);
      return buffer;
    case TangoValueKind::Integer:
      std::snprintf(buffer, sizeof(buffer), "%.0f", a.number);
      return buffer;
    case TangoValueKind::Boolean:
      return a.number != 0.0 ? "true" : "false";
    case TangoValueKind::Enum: {
      const auto index = static_cast<size_t>(a.number);
      if (a.number >= 0.0 && index < a.enum_labels.size()) {
        return a.enum_labels[index];
      }
      std::snprintf(buffer, sizeof(buffer), "%.0f", a.number);
      return buffer;
    }
    case TangoValueKind::Text:
      return a.text;
  }
  return "";
}

// An input for one writable attribute. Send gets the new value as text when the user ends an edit of the input
// (Enter, Tab, or a click outside it).
template <typename Send>
void TangoAttributeInput(const char *id, const TangoAttributeState &a, TangoEditBuffers &buffers, Send send) {
  const std::string key = std::string(id) + "/" + a.name;
  const std::string label = "##" + key;
  bool &active = buffers.active[key];
  ImGui::SetNextItemWidth(-FLT_MIN);
  switch (a.kind) {
    case TangoValueKind::Number:
    case TangoValueKind::Integer: {
      double &value = buffers.numbers[key];
      if (!active) {
        value = a.number;
      }
      const char *format = a.kind == TangoValueKind::Number ? "%.6g" : "%.0f";
      ImGui::InputDouble(label.c_str(), &value, 0.0, 0.0, format);
      active = ImGui::IsItemActive();
      if (ImGui::IsItemDeactivatedAfterEdit()) {
        char buffer[64];
        std::snprintf(buffer, sizeof(buffer), a.kind == TangoValueKind::Number ? "%.17g" : "%.0f", value);
        send(buffer);
      }
      break;
    }
    case TangoValueKind::Boolean: {
      bool value = a.number != 0.0;
      if (ImGui::Checkbox(label.c_str(), &value)) {
        send(value ? "1" : "0");
      }
      break;
    }
    case TangoValueKind::Enum: {
      const std::string preview = a.valid ? TangoValueText(a) : "";
      if (ImGui::BeginCombo(label.c_str(), preview.c_str())) {
        for (size_t i = 0; i < a.enum_labels.size(); i++) {
          if (ImGui::Selectable(a.enum_labels[i].c_str(), static_cast<size_t>(a.number) == i)) {
            send(std::to_string(i));
          }
        }
        ImGui::EndCombo();
      }
      break;
    }
    case TangoValueKind::Text: {
      std::array<char, 256> &value = buffers.texts[key];
      if (!active) {
        std::snprintf(value.data(), value.size(), "%s", a.text.c_str());
      }
      ImGui::InputText(label.c_str(), value.data(), value.size());
      active = ImGui::IsItemActive();
      if (ImGui::IsItemDeactivatedAfterEdit()) {
        send(value.data());
      }
      break;
    }
  }
}
