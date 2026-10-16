#pragma once

#include <algorithm>

#include "algorithms/Dither.hpp"
#include "core/Commands.hpp"
#include "core/Whiteboard.hpp"
#include "gui/GuiWidgets.hpp"
#include "gui/PlotHistory.hpp"
#include "imgui.h"
#include "lib/implot/implot.h"

// The OPD loop of the PLC, the dither that the PLC adds to the delay line, and the loop that looks for the setpoint
// of the smallest intensity. The plot under the tabs shows the OPD and the setpoint of every PLC sample.
class OpdPanel {
 public:
  OpdPanel(CommandBox &command_box, const PlotHistory &plot_history) : commands(command_box), history(plot_history) {
    dither.period_ns = dither::period_from_frequency(dither_frequency_hz);
    dither.amplitude = 0.05f;
  }

  void Render(const Snapshot &snap) {
    if (ImGui::BeginTabBar("##OPD tabs")) {
      if (ImGui::BeginTabItem("Control")) {
        ControlTab(snap);
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Dither")) {
        DitherTab(snap);
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Seeker")) {
        SeekerTab(snap);
        ImGui::EndTabItem();
      }
      ImGui::EndTabBar();
    }
    Plot();
  }

 private:
  CommandBox &commands;
  const PlotHistory &history;

  // The panel owns what it sends, thus the dither tab and the seeker tab always agree on what the core has. The
  // frequency is what the user asks for, and the period is what the PLC gets.
  DitherSettings dither;
  float dither_frequency_hz = 5.0f;
  OpdSeekerCommands seeker_commands;
  bool seeker_run_pending = false;  // the core has not read the newest press of the Start or Stop button

  void ControlTab(const Snapshot &snap) {
    static int mode = 0;
    static float setpoint_um = 0.0f;
    static float open_loop_cmd_um = 0.0f;
    static float kp = 0.0f;
    static float ki = 1.0f;

    bool mode_changed = false;
    ImGui::Text("Control mode:");
    ImGui::SameLine();
    mode_changed |= ImGui::RadioButton("Off##OPD", &mode, 0);
    ImGui::SameLine();
    mode_changed |= ImGui::RadioButton("Open loop##OPD", &mode, 1);
    ImGui::SameLine();
    mode_changed |= ImGui::RadioButton("Closed loop##OPD", &mode, 3);
    if (mode_changed) {
      commands.edit([](Commands &c) { c.opd.mode = mode; });
    }

    // The field shows the setpoint that the core holds, thus it follows the seeker while it runs and it keeps what
    // the seeker found when it stops. It stops following only while the user works on it.
    static bool editing = false;
    const bool seeking = snap.opd_seeker.seeker.running;
    if (!editing) {
      setpoint_um = snap.opd.setpoint_um;
    }
    ImGui::BeginDisabled(seeking);
    if (ImGui::DragFloat("OPD Setpoint", &setpoint_um, 1e-4f, -1e3, 1e3, "%.4f um", ImGuiSliderFlags_AlwaysClamp)) {
      commands.edit([](Commands &c) { c.opd.setpoint_um = setpoint_um; });
    }
    editing = ImGui::IsItemActive();
    ImGui::EndDisabled();
    if (seeking) {
      ImGui::SameLine();
      ImGui::TextDisabled("(the seeker runs)");
    }

    if (ImGui::DragFloat("Open loop DL command", &open_loop_cmd_um, 1e-4f, 0.0f, 15.0f, "%.4f um",
                         ImGuiSliderFlags_AlwaysClamp)) {
      commands.edit([](Commands &c) { c.opd.open_loop_cmd_um = open_loop_cmd_um; });
    }
    if (ImGui::SliderFloat("P##OPD", &kp, 1e-4f, 1e0f, "%.5f", ImGuiSliderFlags_Logarithmic)) {
      commands.edit([](Commands &c) { c.opd.kp = kp; });
    }
    if (ImGui::SliderFloat("I##OPD", &ki, 1e-1f, 1e3f, "%.5f", ImGuiSliderFlags_Logarithmic)) {
      commands.edit([](Commands &c) { c.opd.ki = ki; });
    }
    if (ImGui::Button("Reset phase unwrap")) {
      commands.edit([](Commands &c) { c.opd.reset_unwrap_count++; });
    }

    ImGui::Text("OPD %.4f um, delay line %.4f um, command %.4f um", snap.opd.opd_um, snap.opd.dl_pos_um,
                snap.opd.dl_cmd_um);
  }

  // The sine that the PLC adds to the command of the delay line. The seeker needs it, and the Start button of the
  // seeker turns it on.
  void DitherTab(const Snapshot &snap) {
    bool changed = false;
    ImGui::Text("Dither:");
    ImGui::SameLine();
    changed |= ImGui::RadioButton("Off##Dither", &dither.mode, DitherSettings::kOff);
    ImGui::SameLine();
    changed |= ImGui::RadioButton("Sine##Dither", &dither.mode, DitherSettings::kSine);

    // The PLC takes the period, thus the frequency that it gives back is the one of the nearest whole nanosecond.
    ImGui::SetNextItemWidth(200);
    if (ImGui::DragFloat("Frequency##Dither", &dither_frequency_hz, 1e-2f, 0.05f, 500.0f, "%.3f Hz",
                         ImGuiSliderFlags_AlwaysClamp | ImGuiSliderFlags_Logarithmic)) {
      dither.period_ns = dither::period_from_frequency(dither_frequency_hz);
      changed = true;
    }
    changed |= ImGui::DragFloat("Amplitude##Dither", &dither.amplitude, 1e-4f, 0.0f, 1.0f, "%.4f um",
                                ImGuiSliderFlags_AlwaysClamp);
    changed |= ImGui::DragFloat("Phase at t = 0##Dither", &dither.phase_rad, 1e-3f, -6.2832f, 6.2832f, "%.4f rad",
                                ImGuiSliderFlags_AlwaysClamp);
    if (changed) {
      commands.edit([this](Commands &c) { c.opd.dither = dither; });
    }

    const double frequency_hz = dither::frequency_from_period(dither.period_ns);
    ImGui::Text("Period %lld ns, %.6f Hz", (long long)dither.period_ns, frequency_hz);
    if (snap.camera.framerate > 0.0 && 2.0 * frequency_hz > snap.camera.framerate) {
      ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.0f, 1.0f),
                         "The camera runs at %.1f Hz. It needs more than two frames of each period of the dither.",
                         snap.camera.framerate);
    }
  }

  // The extremum seeker. It makes one photometry region dark by moving the OPD setpoint.
  void SeekerTab(const Snapshot &snap) {
    const ExtremumSeekerState &state = snap.opd_seeker.seeker;
    ExtremumSeekerConfig &seeker = seeker_commands.seeker;

    // The core clears the run command when the seeker cannot run, thus the button follows the core. It shows what
    // the user asked for until the core has read that press.
    if (snap.opd_seeker.run == seeker_commands.run) {
      seeker_run_pending = false;
    } else if (!seeker_run_pending) {
      seeker_commands.run = snap.opd_seeker.run;
    }

    if (!seeker_commands.run) {
      if (ImGui::Button("Start##OpdSeeker")) {
        seeker_commands.run = true;
        seeker_run_pending = true;
        dither.mode = DitherSettings::kSine;
        commands.edit([this](Commands &c) {
          c.opd_seeker = seeker_commands;
          c.opd.dither = dither;
        });
      }
    } else if (ImGui::Button("Stop##OpdSeeker")) {
      seeker_commands.run = false;
      seeker_run_pending = true;
      dither.mode = DitherSettings::kOff;
      commands.edit([this](Commands &c) {
        c.opd_seeker = seeker_commands;
        c.opd.dither = dither;
      });
    }
    ImGui::SameLine();
    gui::Status("Seeking", state.running);
    ImGui::SameLine();
    ImGui::TextDisabled("The Start button also turns the dither on.");

    const ImVec4 warning(1.0f, 0.6f, 0.0f, 1.0f);
    if (snap.opd_seeker.block != kSeekerReady) {
      ImGui::TextColored(warning, "%s", text_of(snap.opd_seeker.block));
    } else if (state.stale) {
      ImGui::TextColored(warning, "No photometry arrives. The setpoint holds.");
    } else if (state.settling) {
      ImGui::TextDisabled("The low pass settles. The setpoint holds.");
    } else {
      ImGui::TextDisabled("The seeker has all it needs.");
    }

    bool changed = false;
    int region = seeker_commands.region + 1;
    ImGui::SetNextItemWidth(200);
    if (ImGui::InputInt("Photometry region##OpdSeeker", &region, 1, 1)) {
      seeker_commands.region = std::clamp(region - 1, 0, kMaxPhotRegions - 1);
      changed = true;
    }
    ImGui::Text("Look for the:");
    ImGui::SameLine();
    changed |= ImGui::RadioButton("Minimum##OpdSeeker", &seeker.direction, kMinimum);
    ImGui::SameLine();
    changed |= ImGui::RadioButton("Maximum##OpdSeeker", &seeker.direction, kMaximum);
    changed |= ImGui::DragFloat("P##OpdSeeker", &seeker.kp, 1e-3f, -1e2f, 1e2f, "%.4f", ImGuiSliderFlags_AlwaysClamp);
    changed |=
        ImGui::DragFloat("I##OpdSeeker", &seeker.ki, 1e-3f, -1e2f, 1e2f, "%.4f per s", ImGuiSliderFlags_AlwaysClamp);
    changed |= ImGui::DragFloat("Demodulation phase##OpdSeeker", &seeker.demod_phase_rad, 1e-3f, -6.2832f, 6.2832f,
                                "%.4f rad", ImGuiSliderFlags_AlwaysClamp);
    changed |= ImGui::DragFloat("Low pass##OpdSeeker", &seeker.lowpass_tau_s, 1e-3f, 1e-2f, 1e2f, "%.3f s",
                                ImGuiSliderFlags_AlwaysClamp | ImGuiSliderFlags_Logarithmic);
    changed |=
        ImGui::DragFloat("Limit##OpdSeeker", &seeker.limit, 1e-3f, 0.0f, 1e2f, "%.3f um", ImGuiSliderFlags_AlwaysClamp);
    changed |= ImGui::DragFloat("Fastest move##OpdSeeker", &seeker.max_rate, 1e-3f, 1e-3f, 1e2f, "%.3f um per s",
                                ImGuiSliderFlags_AlwaysClamp | ImGuiSliderFlags_Logarithmic);
    changed |= ImGui::Checkbox("Divide the gradient by the mean intensity##OpdSeeker", &seeker.normalise);
    if (changed) {
      commands.edit([this](Commands &c) { c.opd_seeker = seeker_commands; });
    }

    ImGui::Text("Intensity %.4e, gradient %+.4e per um", state.mean, state.gradient);
    const char *held = state.at_limit ? " (at the limit)" : (state.at_max_rate ? " (as fast as it may)" : "");
    ImGui::Text("Setpoint %.4f um, %+.4f um from the start%s, %llu photometry samples", state.output, state.offset,
                held, (unsigned long long)state.sample_count);
  }

  void Plot() {
    static float history_length = 10.0f;
    ImGui::SliderFloat("History length##OPD control", &history_length, 0.1f, 10.0f, "%.2f s",
                       ImGuiSliderFlags_Logarithmic);
    if (ImPlot::BeginPlot("##OPD control", ImVec2(-1, 250 * ImGui::GetIO().FontGlobalScale))) {
      gui::SetupTimePlot(history.plc_time, history_length);
      gui::PlotSeries("OPD (um)", history.opd_buffer, 1, 1.0f);
      gui::PlotSeries("Setpoint (um)", history.opd_setpoint_buffer, 3, 2.0f);
      ImPlot::EndPlot();
    }
  }
};
