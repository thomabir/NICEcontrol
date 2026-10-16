#pragma once

#include "core/Core.hpp"
#include "gui/ClockPanel.hpp"
#include "gui/GuiWidgets.hpp"
#include "imgui.h"

// The cycle time of the core, the connection status of each subsystem, and the DC clock.
class CorePanel {
 public:
  void Render(const Snapshot &snap) const {
    const CoreState &state = snap.core;
    ImGui::Text("Cycle %llu at %.3f s, %.2f ms of the %lld ms period, %llu overruns", (unsigned long long)state.cycle,
                state.time_s, state.cycle_ms, (long long)Core::kCyclePeriod.count(),
                (unsigned long long)state.overruns);
    ImGui::Text(
        "Clock %.2f ms, metrology %.2f ms, PLC %.2f ms, tip/tilt %.2f ms, camera %.2f ms, devices %.2f ms, OPD "
        "seeker %.2f ms",
        state.clock_ms, state.metrology_ms, state.plc_ms, state.tiptilt_ms, state.camera_ms, state.devices_ms,
        state.opd_seeker_ms);

    gui::Status("DC clock", snap.clock.clock_present);
    ImGui::SameLine();
    gui::Status("Metrology socket", snap.metrology.socket_open);
    ImGui::SameLine();
    gui::Status("PLC", snap.opd.connected);
    ImGui::SameLine();
    gui::Status("Tip/tilt", snap.tiptilt.connected);
    ImGui::SameLine();
    gui::Status("Camera", snap.camera.connected);
    ImGui::SameLine();
    gui::Status("Shutter", snap.shutter.connected);
    ImGui::SameLine();
    gui::Status("ND filter", snap.ndfilter.connected);
    ImGui::SameLine();
    gui::Status("Polarizer", snap.polarizer.connected);
    ImGui::SameLine();
    gui::Status("Chopper", snap.chopper.connected);

    ImGui::Text("Metrology samples %llu, PLC samples %llu with %llu gaps, camera frames %llu",
                (unsigned long long)snap.metrology.sample_count, (unsigned long long)snap.opd.sample_count,
                (unsigned long long)snap.opd.gaps, (unsigned long long)snap.camera.frame_count);

    if (ImGui::TreeNode("DC clock")) {
      clock.Render(snap);
      ImGui::TreePop();
    }
  }

 private:
  ClockPanel clock;
};
