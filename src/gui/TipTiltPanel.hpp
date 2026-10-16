#pragma once

#include <algorithm>
#include <array>
#include <cfloat>
#include <cstdio>

#include "core/Commands.hpp"
#include "core/Whiteboard.hpp"
#include "gui/GuiWidgets.hpp"
#include "imgui.h"

// The setpoints and the sensors of the tip/tilt actuators, in two tables. The table of beam coordinates is for
// operation, and the table of actuator axes is for diagnosis. Both tables show the same setpoint, which is in local
// axes, thus an edit in one table also changes the other table. The core sends nothing to an actuator until the first
// edit of its setpoint.
class TipTiltPanel {
 public:
  explicit TipTiltPanel(CommandBox &command_box) : commands(command_box) {}

  void Render(const Snapshot &snap) {
    static int mode = 0;
    static std::array<TipTiltSetpoint, kTipTiltActuators> setpoints = [] {
      std::array<TipTiltSetpoint, kTipTiltActuators> start;
      for (int i = 0; i < kTipTiltActuators; i++) {
        start[i].local_mrad = tiptilt::kStartSetpoint[i];
      }
      return start;
    }();
    auto send = [this](int i, const tiptilt::Vector2 &local_mrad) {
      setpoints[i] = {true, local_mrad};
      commands.edit([i, value = setpoints[i]](Commands &c) { c.tiptilt.setpoints[i] = value; });
    };

    ImGui::Text("Control mode:");
    ImGui::SameLine();
    bool mode_changed = ImGui::RadioButton("Open loop##TipTilt", &mode, 0);
    ImGui::SameLine();
    ImGui::BeginDisabled();  // the closed loop with the metrology is not available
    mode_changed |= ImGui::RadioButton("Closed loop##TipTilt", &mode, 1);
    ImGui::EndDisabled();
    ImGui::SameLine();
    mode_changed |= ImGui::RadioButton("Hold still##TipTilt", &mode, 2);
    if (mode_changed) {
      commands.edit([](Commands &c) { c.tiptilt.mode = mode; });
    }

    // A setpoint in beam coordinates goes to the local axes and is clamped there to the range of the actuator.
    ImGui::SeparatorText("Beam coordinates");
    if (BeginTable("##TipTiltBeam", {"Setpoint x", "Setpoint y", "Sensor x", "Sensor y"})) {
      for (int i = 0; i < kTipTiltActuators; i++) {
        const TipTiltActuatorState &state = snap.tiptilt.actuators[i];
        tiptilt::Vector2 beam_mrad = tiptilt::multiply(tiptilt::kLocalToBeam[i], setpoints[i].local_mrad);
        const tiptilt::Vector2 sensor_beam_mrad = tiptilt::multiply(tiptilt::kLocalToBeam[i], state.sensor_local_mrad);
        if (Row(i, state.connected, beam_mrad, nullptr, nullptr, sensor_beam_mrad)) {
          tiptilt::Vector2 local_mrad = tiptilt::multiply(tiptilt::kBeamToLocal[i], beam_mrad);
          for (int c = 0; c < 2; c++) {
            local_mrad[c] = std::clamp(local_mrad[c], state.min_local_mrad[c], state.max_local_mrad[c]);
          }
          send(i, local_mrad);
        }
      }
      ImGui::EndTable();
    }

    ImGui::SeparatorText("Actuator axes");
    if (BeginTable("##TipTiltLocal", {"Setpoint 0", "Setpoint 1", "Sensor 0", "Sensor 1"})) {
      for (int i = 0; i < kTipTiltActuators; i++) {
        const TipTiltActuatorState &state = snap.tiptilt.actuators[i];
        tiptilt::Vector2 local_mrad = setpoints[i].local_mrad;
        if (Row(i, state.connected, local_mrad, &state.min_local_mrad, &state.max_local_mrad,
                state.sensor_local_mrad)) {
          send(i, local_mrad);
        }
      }
      ImGui::EndTable();
    }
  }

 private:
  CommandBox &commands;

  // Starts a table of the tip/tilt actuators with the columns "Actuator", "Status" and the four given value columns.
  // Each column has the width of its widest content, thus the table does not move when the values change.
  static bool BeginTable(const char *id, const std::array<const char *, 4> &value_headers) {
    if (!ImGui::BeginTable(id, 6, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg)) {
      return false;
    }
    const float frame_padding = 2.0f * ImGui::GetStyle().FramePadding.x;
    const float value_width = ImGui::CalcTextSize("-88.888 mrad").x;
    const char *headers[6] = {"Actuator",       "Status",         value_headers[0],
                              value_headers[1], value_headers[2], value_headers[3]};
    const float widths[6] = {ImGui::CalcTextSize("Actuator").x,
                             ImGui::CalcTextSize("not connected").x,
                             value_width + frame_padding,
                             value_width + frame_padding,
                             value_width,
                             value_width};
    for (int column = 0; column < 6; column++) {
      const float width = std::max(widths[column], ImGui::CalcTextSize(headers[column]).x);
      ImGui::TableSetupColumn(headers[column], ImGuiTableColumnFlags_WidthFixed, width);
    }
    ImGui::TableHeadersRow();
    return true;
  }

  // One row of a tip/tilt table: the name, the status, two setpoint fields and two sensor values. Without min and max,
  // the setpoint fields do not clamp. Returns true if the user changed the setpoint.
  static bool Row(int i, bool connected, tiptilt::Vector2 &setpoint_mrad, const tiptilt::Vector2 *min_mrad,
                  const tiptilt::Vector2 *max_mrad, const tiptilt::Vector2 &sensor_mrad) {
    ImGui::PushID(i);
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(kTipTiltActuatorNames[i]);
    ImGui::TableNextColumn();
    gui::Status(connected ? "connected" : "not connected", connected);

    ImGui::BeginDisabled(!connected);
    bool changed = false;
    for (int c = 0; c < 2; c++) {
      ImGui::PushID(c);
      ImGui::TableNextColumn();
      ImGui::SetNextItemWidth(-FLT_MIN);
      changed |= ImGui::DragScalar("##setpoint", ImGuiDataType_Double, &setpoint_mrad[c], 0.001f,
                                   min_mrad ? &(*min_mrad)[c] : nullptr, max_mrad ? &(*max_mrad)[c] : nullptr,
                                   "%.3f mrad", min_mrad ? ImGuiSliderFlags_AlwaysClamp : ImGuiSliderFlags_None);
      ImGui::PopID();
    }
    ImGui::EndDisabled();

    // The sensor values are right aligned, thus a change of sign does not move the digits.
    for (int c = 0; c < 2; c++) {
      ImGui::TableNextColumn();
      char text[32];
      std::snprintf(text, sizeof(text), "%.3f mrad", sensor_mrad[c]);
      ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x - ImGui::CalcTextSize(text).x);
      ImGui::TextUnformatted(text);
    }
    ImGui::PopID();
    return changed;
  }
};
