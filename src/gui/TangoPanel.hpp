#pragma once

#include <unistd.h>

#include <string>
#include <vector>

#include "core/Commands.hpp"
#include "core/Whiteboard.hpp"
#include "gui/GuiWidgets.hpp"
#include "gui/TangoDeviceWidgets.hpp"
#include "imgui.h"
#include "util/ScriptRunner.hpp"

// The Tango devices, and the restart of the Tango database and the device servers by the script of the TangoServers
// repository.
class TangoPanel {
 public:
  explicit TangoPanel(CommandBox &command_box) : commands(command_box) {}

  // Buttons that restart the Tango database and the device servers of all users, and the output of the restart.
  void RenderRestartBar() {
    restart.poll();
    const bool running = restart.running();
    ImGui::BeginDisabled(running);
    if (ImGui::Button("Restart Tango and device servers")) {
      restart_database = true;
      ImGui::OpenPopup("Restart Tango");
    }
    ImGui::SameLine();
    if (ImGui::Button("Restart device servers")) {
      restart_database = false;
      ImGui::OpenPopup("Restart Tango");
    }
    ImGui::EndDisabled();

    if (ImGui::BeginPopupModal("Restart Tango", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::Text("This stops the device servers in %s/servers.conf, also of other users.", NICE_TANGO_SERVERS_DIR);
      if (restart_database) {
        ImGui::TextUnformatted("Then it restarts the Tango database.");
      }
      ImGui::TextUnformatted("Then it starts the device servers.");
      if (ImGui::Button("Restart")) {
        std::vector<std::string> args{std::string(NICE_TANGO_SERVERS_DIR) + "/restart_servers.sh"};
        if (!restart_database) {
          args.push_back("--keep-database");
        }
        restart.start(args);
        ImGui::CloseCurrentPopup();
      }
      ImGui::SameLine();
      if (ImGui::Button("Cancel")) {
        ImGui::CloseCurrentPopup();
      }
      ImGui::EndPopup();
    }

    if (!restart.started()) {
      return;
    }
    ImGui::SameLine();
    if (running) {
      ImGui::TextUnformatted("Restart in progress");
    } else if (restart.status() == 0) {
      gui::Status("Restart complete", true);
    } else {
      gui::Status("Restart failed", false);
    }
    ImGui::SameLine();
    if (ImGui::TreeNode("Restart output")) {
      ImGui::TextUnformatted(restart.output().c_str());
      ImGui::TreePop();
    }
  }

  void RenderDevices(const Snapshot &snap) {
    DeviceRow("Shutter", "Shutter", snap.shutter, [](Commands &c) -> TangoDeviceCommands & { return c.shutter; });
    DeviceRow("ND Filter", "NDFilter", snap.ndfilter, [](Commands &c) -> TangoDeviceCommands & { return c.ndfilter; });
    DeviceRow("Polarizer", "Polarizer", snap.polarizer,
              [](Commands &c) -> TangoDeviceCommands & { return c.polarizer; });
    DeviceRow("Chopper", "Chopper", snap.chopper, [](Commands &c) -> TangoDeviceCommands & { return c.chopper; });
  }

 private:
  CommandBox &commands;
  TangoEditBuffers edit;
  ScriptRunner restart{"/tmp/nicecontrol-tango-restart-" + std::to_string(getuid()) + ".log"};
  bool restart_database = true;

  // One line with the device name and a button for each command that the device reports. The table of the scalar
  // attributes shows when the user expands the line.
  template <typename Select>
  void DeviceRow(const char *label, const char *id, const TangoDeviceState &state, Select select) {
    ImGui::AlignTextToFramePadding();
    const bool open = ImGui::TreeNode((std::string(label) + "##tango " + id).c_str());
    ImGui::SameLine();
    if (!state.connected) {
      ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.0f, 1.0f), "Not connected. The core retries.");
    } else {
      for (const std::string &command : state.device_commands) {
        if (ImGui::Button((command + "##" + id).c_str())) {
          commands.edit([&](Commands &c) {
            TangoDeviceCommands &target = select(c);
            target.device_command = command;
            target.device_command_count++;
          });
        }
        ImGui::SameLine();
      }
      ImGui::NewLine();
      if (!state.last_error.empty()) {
        ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.0f, 1.0f), "Error: %s", state.last_error.c_str());
      }
    }
    if (open) {
      if (state.connected) {
        AttributeTable(id, state, select);
      }
      ImGui::TreePop();
    }
  }

  // A table of the scalar attributes, with an input for each writable attribute.
  template <typename Select>
  void AttributeTable(const char *id, const TangoDeviceState &state, Select select) {
    if (state.attributes.empty() || !ImGui::BeginTable((std::string("##attributes ") + id).c_str(), 4,
                                                       ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
      return;
    }
    ImGui::TableSetupColumn("Attribute");
    ImGui::TableSetupColumn("Value");
    ImGui::TableSetupColumn("Unit");
    ImGui::TableSetupColumn("Set");
    ImGui::TableHeadersRow();
    for (const TangoAttributeState &a : state.attributes) {
      ImGui::TableNextRow();
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(a.name.c_str());
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(a.valid ? TangoValueText(a).c_str() : "-");
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(a.unit.c_str());
      ImGui::TableNextColumn();
      if (a.writable) {
        TangoAttributeInput(id, a, edit, [&](const std::string &value) {
          commands.edit([&](Commands &c) {
            TangoDeviceCommands &target = select(c);
            target.write_attribute = a.name;
            target.write_value = value;
            target.write_count++;
          });
        });
      }
    }
    ImGui::EndTable();
  }
};
