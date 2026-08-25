#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

// GUI
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h>

#include "algorithms/Dither.hpp"
#include "algorithms/FftCalculator.hpp"
#include "client/nice_clock.h"
#include "core/Core.hpp"
#include "data/PhotometryRegions.hpp"
#include "data/ScrollingBufferT.hpp"
#include "lib/fonts/SourceSans3Regular.cpp"
#include "lib/implot/implot.h"

// Windows
#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

// The user interface. It shows what the core publishes and it sends commands to the core. It never touches hardware,
// so the program keeps running with no user interface open.
class NiceGui {
 public:
  explicit NiceGui(Core &core)
      : core(core),
        adc_reader(core.whiteboard().adc.subscribe()),
        plc_reader(core.whiteboard().plc.subscribe()),
        phot_reader(core.whiteboard().phot.subscribe()) {
    phot_buffers.resize(kMaxPhotRegions, ScrollingBufferT<double, double>(kPhotHistoryPoints));
    phot_intensity_of_one.resize(kMaxPhotRegions, 1.0f);
    opd_dither.period_ns = dither::period_from_frequency(opd_dither_frequency_hz);
    opd_dither.amplitude = 0.05f;
  }

  ~NiceGui() { Cleanup(); }

  void start() {
    gui_thread = std::jthread([this]() {
      // GLFW is preferably at home in a single thread, so the whole window lives here.
      if (!Initialize()) {
        std::cerr << "NiceGui: cannot initialise the window." << std::endl;
        return;
      }
      while (!glfwWindowShouldClose(window)) {
        RenderFrame();
      }
    });
  }

  void wait_for_close() { gui_thread.join(); }

 private:
  static constexpr int kPhotHistoryPoints = 20000;

  Core &core;
  Consumer *adc_reader;
  Consumer *plc_reader;
  Consumer *phot_reader;

  Snapshot snap;
  std::jthread gui_thread;
  GLFWwindow *window = nullptr;
  const char *glsl_version = nullptr;
  ImGuiIO *io = nullptr;
  ImFont *mainFont = nullptr;
  ImVec4 clear_color;

  // Plot history. The core owns the streams and this is the copy that ImPlot draws from.
  ScrollingBufferT<int, int> adc_buffers[16];
  int adc_time = 0;
  ScrollingBufferT<double, double> dl_pos_buffer;
  ScrollingBufferT<double, double> dl_cmd_buffer;
  ScrollingBufferT<double, double> opd_buffer;
  ScrollingBufferT<double, double> opd_setpoint_buffer;
  ScrollingBufferT<double, double> qpd_buffers[12];
  double plc_time = 0.0;
  std::vector<ScrollingBufferT<double, double>> phot_buffers;
  std::vector<float> phot_intensity_of_one;
  double phot_time = 0.0;

  // OPD panel state. The panel owns what it sends, thus the dither tab and the seeker tab always agree on what the
  // core has. The frequency is what the user asks for, and the period is what the PLC gets.
  DitherSettings opd_dither;
  float opd_dither_frequency_hz = 5.0f;
  OpdSeekerCommands opd_seeker;

  // Camera panel state. The rectangles on screen become the regions that the camera measures.
  bool camera_seeded = false;
  int camera_region_count = 1;
  std::vector<ImPlotRect> camera_rects;
  bool camera_rects_from_device = false;
  bool camera_subtract_background = false;
  std::vector<PhotRegion> camera_sent_regions;
  Image<int> camera_image;

  template <typename F>
  void Command(F change) {
    core.commands().edit(change);
  }

  void RenderFrame() {
    snap = core.whiteboard().snapshot();
    DrainStreams();

    glfwPollEvents();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::PushFont(mainFont);
    RenderUI();
    ImGui::PopFont();

    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w,
                 clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    if (io->ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
      GLFWwindow *backup_current_context = glfwGetCurrentContext();
      ImGui::UpdatePlatformWindows();
      ImGui::RenderPlatformWindowsDefault();
      glfwMakeContextCurrent(backup_current_context);
    }

    glfwSwapBuffers(window);
  }

  // Every stream is drained in every frame, whether or not its panel is open. A stream that nobody drains overflows.
  void DrainStreams() {
    Whiteboard &wb = core.whiteboard();

    Measurement<AdcSample> adc_measurement;
    while (wb.adc.try_pop(adc_reader, adc_measurement)) {
      const AdcSample &adc_sample = adc_measurement.value;
      adc_time = adc_sample.counter;
      for (int channel = 0; channel < 16; channel++) {
        adc_buffers[channel].AddPoint(adc_sample.counter, adc_sample.value[channel]);
      }
    }

    Measurement<PlcSample> plc_measurement;
    while (wb.plc.try_pop(plc_reader, plc_measurement)) {
      const PlcSample &plc_sample = plc_measurement.value;
      plc_time = plc_measurement.time.t_DC * 1e-9;
      dl_pos_buffer.AddPoint(plc_time, plc_sample.dl_pos_um);
      dl_cmd_buffer.AddPoint(plc_time, plc_sample.dl_cmd_um);
      opd_buffer.AddPoint(plc_time, plc_sample.opd_um);
      // The PLC does not send the setpoint back, thus the plot takes the value that the core last sent.
      opd_setpoint_buffer.AddPoint(plc_time, snap.opd.setpoint_um);
      const QpdData &qpd1 = plc_sample.qpd1;
      const QpdData &qpd2 = plc_sample.qpd2;
      const double qpd_values[12] = {qpd1.x1, qpd1.y1, qpd1.i1, qpd1.x2, qpd1.y2, qpd1.i2,
                                     qpd2.x1, qpd2.y1, qpd2.i1, qpd2.x2, qpd2.y2, qpd2.i2};
      for (int i = 0; i < 12; i++) {
        qpd_buffers[i].AddPoint(plc_time, qpd_values[i]);
      }
    }

    Measurement<PhotSample> phot_measurement;
    while (wb.phot.try_pop(phot_reader, phot_measurement)) {
      const PhotSample &phot_sample = phot_measurement.value;
      phot_time = phot_measurement.time.t_DC * 1e-9;
      for (int region = 0; region < kMaxPhotRegions; region++) {
        phot_buffers[region].AddPoint(phot_time, phot_sample.values[region]);
      }
    }
  }

  void RenderUI() {
    static ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings;

    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);

    ImGui::Begin("NICE Control", nullptr, flags);
    ImGuiIO &frame_io = ImGui::GetIO();

    static bool show_demo_window = false;
    static bool show_app_metrics = false;

    if (ImGui::CollapsingHeader("Core", ImGuiTreeNodeFlags_DefaultOpen)) {
      WindowCore();
    }
    if (ImGui::CollapsingHeader("Shutter")) {
      WindowTangoDevice("Shutter", snap.shutter, [](Commands &c) -> TangoDeviceCommands & { return c.shutter; });
    }
    if (ImGui::CollapsingHeader("ND Filter")) {
      WindowTangoDevice("NDFilter", snap.ndfilter, [](Commands &c) -> TangoDeviceCommands & { return c.ndfilter; });
    }
    if (ImGui::CollapsingHeader("Flir Camera")) {
      WindowFlirCam();
    }
    if (ImGui::CollapsingHeader("ADC Measurements")) {
      WindowAdc(frame_io);
    }
    if (ImGui::CollapsingHeader("EtherCAT Monitor")) {
      WindowEthercatMonitor();
    }
    if (ImGui::CollapsingHeader("OPD")) {
      WindowOpd();
    }
    if (ImGui::CollapsingHeader("Lateral beam control")) {
      WindowBeamControl();
    }
    if (ImGui::CollapsingHeader("Program settings")) {
      ImGui::DragFloat("GUI scale", &frame_io.FontGlobalScale, 0.005f, 0.5, 6.0, "%.2f", ImGuiSliderFlags_AlwaysClamp);
      ImGui::Checkbox("Show demo windows", &show_demo_window);
      ImGui::Checkbox("Show app metrics", &show_app_metrics);
    }
    ImGui::End();

    if (show_demo_window) {
      ImGui::ShowDemoWindow();
      ImPlot::ShowDemoWindow();
    }
    if (show_app_metrics) {
      ImGui::ShowMetricsWindow();
    }
  }

  void WindowCore() {
    const CoreState &state = snap.core;
    ImGui::Text("Cycle %llu at %.3f s, %.2f ms of the %lld ms period, %llu overruns", (unsigned long long)state.cycle,
                state.time_s, state.cycle_ms, (long long)Core::kCyclePeriod.count(),
                (unsigned long long)state.overruns);
    ImGui::Text(
        "Clock %.2f ms, metrology %.2f ms, PLC %.2f ms, tip/tilt %.2f ms, camera %.2f ms, devices %.2f ms, OPD "
        "seeker %.2f ms",
        state.clock_ms, state.metrology_ms, state.plc_ms, state.tiptilt_ms, state.camera_ms, state.devices_ms,
        state.opd_seeker_ms);

    Status("DC clock", snap.clock.clock_present);
    ImGui::SameLine();
    Status("Metrology socket", snap.metrology.socket_open);
    ImGui::SameLine();
    Status("PLC", snap.opd.connected);
    ImGui::SameLine();
    Status("Tip/tilt", snap.tiptilt.connected);
    ImGui::SameLine();
    Status("Camera", snap.camera.connected);
    ImGui::SameLine();
    Status("Shutter", snap.shutter.connected);
    ImGui::SameLine();
    Status("ND filter", snap.ndfilter.connected);

    ImGui::Text("Metrology samples %llu, PLC samples %llu with %llu gaps, camera frames %llu",
                (unsigned long long)snap.metrology.sample_count, (unsigned long long)snap.opd.sample_count,
                (unsigned long long)snap.opd.gaps, (unsigned long long)snap.camera.frame_count);

    if (ImGui::TreeNode("DC clock")) {
      WindowClock();
      ImGui::TreePop();
    }
  }

  // The distributed clock of the bus, and the estimate that gives its time for any time of the PC clock.
  void WindowClock() {
    const ClockState &state = snap.clock;
    ImGui::Text("Health:   %s", state.dc_good ? "good, thus " NICE_CLOCK_PATH " gives t_DC to the other programs"
                                              : "no time of the bus, thus t_DC and the shared record are not good");
    if (!state.card_open) {
      ImGui::TextDisabled("The card is not open. The program tries once, at its start.");
      return;
    }
    ImGui::Text("Card:     AL state %d, %llu pairs, age %.1f ms, read span %.1f us", state.al_state,
                (unsigned long long)state.sample_count, state.age_ms, state.read_span_us);
    ImGui::Text("t_DC:     %lld ns", (long long)state.dc_ns);
    if (!state.locked) {
      ImGui::TextDisabled("Estimate: none. The maindevice does not distribute the clock.");
      return;
    }
    ImGui::Text("Offset:   t_DC - t_PC %+.6f s", 1e-9 * static_cast<double>(snap.time.t_DC - snap.time.t_PC));
    ImGui::Text("Estimate: %+.3f ppm +- %.0f ppb, uncertainty %.0f ns", state.rate_ppm, state.rate_sd_ppb,
                state.offset_sd_ns);
    ImGui::Text("          last error %+.0f ns, %llu pairs refused", state.error_ns,
                (unsigned long long)state.rejected_count);
  }

  static void Status(const char *label, bool up) {
    const ImVec4 colour = up ? ImVec4(0.0f, 1.0f, 0.0f, 1.0f) : ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
    ImGui::TextColored(colour, "%s", label);
  }

  // A row of buttons, one for each command that the device reports.
  template <typename Select>
  void WindowTangoDevice(const char *id, const TangoDeviceState &state, Select select) {
    if (!state.connected) {
      ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.0f, 1.0f), "Not connected. The core retries.");
      return;
    }
    ImGui::Text("Commands:");
    ImGui::SameLine();
    for (const std::string &command : state.device_commands) {
      if (ImGui::Button((command + "##" + id).c_str())) {
        Command([&](Commands &c) {
          TangoDeviceCommands &target = select(c);
          target.device_command = command;
          target.device_command_count++;
        });
      }
      ImGui::SameLine();
    }
    ImGui::NewLine();
  }

  // The OPD loop of the PLC, the dither that the PLC adds to the delay line, and the loop that looks for the setpoint
  // of the smallest intensity. The plot under the tabs shows the OPD and the setpoint of every PLC sample.
  void WindowOpd() {
    if (ImGui::BeginTabBar("##OPD tabs")) {
      if (ImGui::BeginTabItem("Control")) {
        OpdControlTab();
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Dither")) {
        OpdDitherTab();
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("Seeker")) {
        OpdSeekerTab();
        ImGui::EndTabItem();
      }
      ImGui::EndTabBar();
    }
    OpdPlot();
  }

  void OpdControlTab() {
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
      Command([](Commands &c) { c.opd.mode = mode; });
    }

    // The seeker owns the setpoint while it runs. The field follows what the seeker found, and it keeps the last
    // value when the seeker stops.
    const bool seeking = snap.opd_seeker.seeker.running;
    if (seeking) {
      setpoint_um = snap.opd.setpoint_um;
    }
    ImGui::BeginDisabled(seeking);
    if (ImGui::DragFloat("OPD Setpoint", &setpoint_um, 1e-4, -1e3, 1e3, "%.4f um", ImGuiSliderFlags_AlwaysClamp)) {
      Command([](Commands &c) { c.opd.setpoint_um = setpoint_um; });
    }
    ImGui::EndDisabled();
    if (seeking) {
      ImGui::SameLine();
      ImGui::TextDisabled("(the seeker runs)");
    }

    if (ImGui::DragFloat("Open loop DL command", &open_loop_cmd_um, 1e-4, 0.0f, 15.0f, "%.4f um",
                         ImGuiSliderFlags_AlwaysClamp)) {
      Command([](Commands &c) { c.opd.open_loop_cmd_um = open_loop_cmd_um; });
    }
    if (ImGui::SliderFloat("P##OPD", &kp, 1e-4f, 1e0f, "%.5f", ImGuiSliderFlags_Logarithmic)) {
      Command([](Commands &c) { c.opd.kp = kp; });
    }
    if (ImGui::SliderFloat("I##OPD", &ki, 1e-1f, 1e3f, "%.5f", ImGuiSliderFlags_Logarithmic)) {
      Command([](Commands &c) { c.opd.ki = ki; });
    }
    if (ImGui::Button("Reset phase unwrap")) {
      Command([](Commands &c) { c.opd.reset_unwrap_count++; });
    }

    ImGui::Text("OPD %.4f um, delay line %.4f um, command %.4f um", snap.opd.opd_um, snap.opd.dl_pos_um,
                snap.opd.dl_cmd_um);
  }

  // The sine that the PLC adds to the command of the delay line. The seeker needs it, and the Start button of the
  // seeker turns it on.
  void OpdDitherTab() {
    bool changed = false;
    ImGui::Text("Dither:");
    ImGui::SameLine();
    changed |= ImGui::RadioButton("Off##Dither", &opd_dither.mode, DitherSettings::kOff);
    ImGui::SameLine();
    changed |= ImGui::RadioButton("Sine##Dither", &opd_dither.mode, DitherSettings::kSine);

    // The PLC takes the period, thus the frequency that it gives back is the one of the nearest whole nanosecond.
    ImGui::SetNextItemWidth(200);
    if (ImGui::DragFloat("Frequency##Dither", &opd_dither_frequency_hz, 1e-2f, 0.05f, 500.0f, "%.3f Hz",
                         ImGuiSliderFlags_AlwaysClamp | ImGuiSliderFlags_Logarithmic)) {
      opd_dither.period_ns = dither::period_from_frequency(opd_dither_frequency_hz);
      changed = true;
    }
    changed |= ImGui::DragFloat("Amplitude##Dither", &opd_dither.amplitude, 1e-4f, 0.0f, 1.0f, "%.4f um",
                                ImGuiSliderFlags_AlwaysClamp);
    changed |= ImGui::DragFloat("Phase at t = 0##Dither", &opd_dither.phase_rad, 1e-3f, -6.2832f, 6.2832f, "%.4f rad",
                                ImGuiSliderFlags_AlwaysClamp);
    if (changed) {
      Command([this](Commands &c) { c.opd.dither = opd_dither; });
    }

    const double frequency_hz = dither::frequency_from_period(opd_dither.period_ns);
    ImGui::Text("Period %lld ns, %.6f Hz", (long long)opd_dither.period_ns, frequency_hz);
    if (snap.camera.framerate > 0.0 && 2.0 * frequency_hz > snap.camera.framerate) {
      ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.0f, 1.0f),
                         "The camera runs at %.1f Hz. It needs more than two frames of each period of the dither.",
                         snap.camera.framerate);
    }
  }

  // The extremum seeker. It makes one photometry region dark by moving the OPD setpoint.
  void OpdSeekerTab() {
    const ExtremumSeekerState &state = snap.opd_seeker.seeker;
    ExtremumSeekerConfig &seeker = opd_seeker.seeker;

    if (!opd_seeker.run) {
      if (ImGui::Button("Start##OpdSeeker")) {
        opd_seeker.run = true;
        opd_dither.mode = DitherSettings::kSine;
        Command([this](Commands &c) {
          c.opd_seeker = opd_seeker;
          c.opd.dither = opd_dither;
        });
      }
    } else if (ImGui::Button("Stop##OpdSeeker")) {
      opd_seeker.run = false;
      opd_dither.mode = DitherSettings::kOff;
      Command([this](Commands &c) {
        c.opd_seeker = opd_seeker;
        c.opd.dither = opd_dither;
      });
    }
    ImGui::SameLine();
    Status("Seeking", state.running);
    ImGui::SameLine();
    ImGui::TextDisabled("The Start button also turns the dither on.");

    bool changed = false;
    int region = opd_seeker.region + 1;
    ImGui::SetNextItemWidth(200);
    if (ImGui::InputInt("Photometry region##OpdSeeker", &region, 1, 1)) {
      opd_seeker.region = std::clamp(region - 1, 0, kMaxPhotRegions - 1);
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
    changed |= ImGui::Checkbox("Divide the gradient by the mean intensity##OpdSeeker", &seeker.normalise);
    if (changed) {
      Command([this](Commands &c) { c.opd_seeker = opd_seeker; });
    }

    ImGui::Text("Intensity %.4e, gradient %+.4e per um", state.mean, state.gradient);
    ImGui::Text("Setpoint %.4f um, %+.4f um from the start%s, %llu photometry samples", state.output, state.offset,
                state.at_limit ? " (at the limit)" : "", (unsigned long long)state.sample_count);
  }

  void OpdPlot() {
    static float history_length = 10.0f;
    ImGui::SliderFloat("History length##OPD control", &history_length, 0.1f, 10.0f, "%.2f s",
                       ImGuiSliderFlags_Logarithmic);
    if (ImPlot::BeginPlot("##OPD control", ImVec2(-1, 250 * io->FontGlobalScale))) {
      SetupTimePlot(history_length);
      PlotSeries("OPD (um)", opd_buffer, 1, 1.0f);
      PlotSeries("Setpoint (um)", opd_setpoint_buffer, 3, 2.0f);
      ImPlot::EndPlot();
    }
  }

  void WindowBeamControl() {
    static int mode = 0;
    static float x1 = 0.0f, y1 = 0.0f, x2 = 0.0f, y2 = 0.0f;

    ImGui::Text("Control mode:");
    ImGui::SameLine();
    bool mode_changed = ImGui::RadioButton("Open loop##TipTilt", &mode, 0);
    ImGui::SameLine();
    mode_changed |= ImGui::RadioButton("Closed loop##TipTilt", &mode, 1);
    ImGui::SameLine();
    mode_changed |= ImGui::RadioButton("Hold still##TipTilt", &mode, 2);
    if (mode_changed) {
      Command([](Commands &c) { c.tiptilt.mode = mode; });
    }

    ImGui::Text("Open loop commands:");
    if (ImGui::DragFloat("X1##TipTilt", &x1, 0.01f, -1000.0f, 1000.0f, "%.2f urad", ImGuiSliderFlags_AlwaysClamp)) {
      Command([](Commands &c) { c.tiptilt.x1 = x1; });
    }
    ImGui::SameLine();
    ImGui::Text("(Encoder: %.0f)", snap.tiptilt.x1);
    if (ImGui::DragFloat("Y1##TipTilt", &y1, 0.01f, -1000.0f, 1000.0f, "%.2f urad", ImGuiSliderFlags_AlwaysClamp)) {
      Command([](Commands &c) { c.tiptilt.y1 = y1; });
    }
    ImGui::SameLine();
    ImGui::Text("(Encoder: %.0f)", snap.tiptilt.y1);
    if (ImGui::DragFloat("X2##TipTilt", &x2, 0.01f, -1000.0f, 1000.0f, "%.2f urad", ImGuiSliderFlags_AlwaysClamp)) {
      Command([](Commands &c) { c.tiptilt.x2 = x2; });
    }
    ImGui::SameLine();
    ImGui::Text("(Encoder: %.0f)", snap.tiptilt.x2);
    if (ImGui::DragFloat("Y2##TipTilt", &y2, 0.01f, -1000.0f, 1000.0f, "%.2f urad", ImGuiSliderFlags_AlwaysClamp)) {
      Command([](Commands &c) { c.tiptilt.y2 = y2; });
    }
    ImGui::SameLine();
    ImGui::Text("(Encoder: %.0f)", snap.tiptilt.y2);
  }

  void WindowAdc(ImGuiIO &frame_io) {
    static float history_length = 1000.f;
    ImGui::SliderFloat("ADC History", &history_length, 200, 70000, "%.0f samples", ImGuiSliderFlags_Logarithmic);

    static const char *labels[16] = {
        "QOD1 UP",        "QPD1 LEFT", "QPD1 RIGHT",     "QPD1 DOWN", "QPD1 sum", "OPD ref",
        "Pos ref (QPD1)", "OPD back",  "Pos ref (QPD2)", "NC3",       "NC4",      "NC5",
        "QPD2 UP",        "QPD2 LEFT", "QPD2 RIGHT",     "QPD2 DOWN"};

    static ImPlotAxisFlags xflags = ImPlotAxisFlags_NoTickLabels;
    static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
    if (ImPlot::BeginPlot("##ADC", ImVec2(-1, 400 * frame_io.FontGlobalScale))) {
      ImPlot::SetupAxes(nullptr, nullptr, xflags, yflags);
      ImPlot::SetupAxisLimits(ImAxis_X1, adc_time - history_length, adc_time, ImGuiCond_Always);
      ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
      ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
      for (int i = 0; i < 16; i++) {
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(i), 2);
        ImPlot::PlotLine(labels[i], &adc_buffers[i].Data[0].time, &adc_buffers[i].Data[0].value,
                         adc_buffers[i].Data.size(), 0, adc_buffers[i].Offset, 2 * sizeof(int));
      }
      ImPlot::EndPlot();
    }
  }

  void WindowEthercatMonitor() {
    if (ImGui::TreeNode("Delay line##EtherCAT")) {
      static float history_length = 10.0f;
      ImGui::SliderFloat("History length##DL", &history_length, 0.1f, 10.0f, "%.2f s", ImGuiSliderFlags_Logarithmic);
      if (ImPlot::BeginPlot("##EtherCAT Delay Line", ImVec2(-1, 200 * io->FontGlobalScale))) {
        SetupTimePlot(history_length);
        PlotSeries("DL position (um)", dl_pos_buffer, 0, 3.0f);
        PlotSeries("DL command (um)", dl_cmd_buffer, 1, 3.0f);
        ImPlot::EndPlot();
      }
      ImGui::TreePop();
    }

    if (ImGui::TreeNode("OPD##EtherCAT")) {
      static float history_length = 10.0f;
      ImGui::SliderFloat("History length##OPD", &history_length, 0.1f, 10.0f, "%.2f s", ImGuiSliderFlags_Logarithmic);
      if (ImPlot::BeginPlot("##EtherCAT OPD", ImVec2(-1, 200 * io->FontGlobalScale))) {
        SetupTimePlot(history_length);
        PlotSeries("OPD (um)", opd_buffer, 1, 1.0f);
        ImPlot::EndPlot();
      }

      const int n_points = 1000;
      static float mean = 0.0f;
      static float stddev = 0.0f;
      if (opd_buffer.Data.size() > n_points + 2) {
        const auto last = opd_buffer.GetLastN(n_points);
        std::vector<double> values;
        values.reserve(n_points);
        for (const auto &m : last) {
          values.push_back(m.value);
        }
        mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
        const double square_sum = std::inner_product(values.begin(), values.end(), values.begin(), 0.0);
        stddev = std::sqrt(square_sum / values.size() - mean * mean);
      }
      ImGui::Text("Last %d samples: Mean: %.4f um, Std: %.4f nm", n_points, mean, stddev * 1e3);
      ImGui::TreePop();
    }

    if (ImGui::TreeNode("OPD FFT##EtherCAT")) {
      const static int fs = 1 / 150e-6;      // the PLC samples every 150 us
      const static int fft_size = 1024 * 8;  // 1.2 s of data
      static double fft_power[fft_size / 2];
      static double fft_freq[fft_size / 2];
      static FFT_calculator<double, double> fft(fft_size, fs, &opd_buffer, fft_power, fft_freq);

      fft.calculate();

      static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
      if (ImPlot::BeginPlot("##FFT", ImVec2(-1, 400 * io->FontGlobalScale))) {
        ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
        ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
        ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_None, yflags);
        ImPlot::SetupAxisLimits(ImAxis_X1, 1, 3333);
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(1), 3);
        ImPlot::PlotLine("OPD PSD (nm/sqrtHz)", &fft_freq[0], &fft_power[0], fft_size / 2);
        ImPlot::EndPlot();
      }
      ImGui::TreePop();
    }

    if (ImGui::TreeNode("QPD##EtherCAT")) {
      static const char *labels[12] = {"QPD1 X1", "QPD1 Y1", "QPD1 I1", "QPD1 X2", "QPD1 Y2", "QPD1 I2",
                                       "QPD2 X1", "QPD2 Y1", "QPD2 I1", "QPD2 X2", "QPD2 Y2", "QPD2 I2"};
      static float history_length = 10.0f;
      ImGui::SliderFloat("History length##QPD", &history_length, 0.1f, 10.0f, "%.2f s", ImGuiSliderFlags_Logarithmic);
      if (ImPlot::BeginPlot("##EtherCAT QPD", ImVec2(-1, 200 * io->FontGlobalScale))) {
        SetupTimePlot(history_length);
        for (int i = 0; i < 12; i++) {
          PlotSeries(labels[i], qpd_buffers[i], i, 1.0f);
        }
        ImPlot::EndPlot();
      }
      ImGui::TreePop();
    }
  }

  void SetupTimePlot(float history_length) {
    static ImPlotAxisFlags xflags = ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoTickLabels;
    static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
    ImPlot::SetupAxes(nullptr, nullptr, xflags, yflags);
    ImPlot::SetupAxisLimits(ImAxis_X1, plc_time - history_length, plc_time, ImGuiCond_Always);
    ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
    ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
  }

  void PlotSeries(const char *label, ScrollingBufferT<double, double> &buffer, int colour, float thickness) {
    ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(colour), thickness * io->FontGlobalScale);
    ImPlot::PlotLine(label, &buffer.Data[0].time, &buffer.Data[0].value, buffer.Data.size(), 0, buffer.Offset,
                     2 * sizeof(double));
  }

  // The FLIR camera, in three parts: Camera settings, Image, Photometry. The camera application owns the device and
  // measures the photometry. This panel draws what it reports and asks it for changes.
  void WindowFlirCam() {
    static ImVec2 window_size(600, 800);
    ImGui::SetNextWindowSize(window_size, ImGuiCond_FirstUseEver);
    ImGui::Begin("FlirCamWindow", nullptr, 0);

    if (!snap.camera.connected) {
      if (ImGui::Button("Connect")) {
        Command([](Commands &c) { c.camera.connect = true; });
      }
      ImGui::SameLine();
      ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "Disconnected");
      camera_seeded = false;
      ImGui::End();
      return;
    }

    if (ImGui::Button("Disconnect")) {
      Command([](Commands &c) { c.camera.connect = false; });
    }
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Connected");

    // The camera reports the truth, so the panel starts from what the core last read out of it.
    if (!camera_seeded && !snap.camera.regions.empty()) {
      camera_seeded = true;
      camera_region_count = static_cast<int>(snap.camera.regions.size());
      camera_sent_regions = snap.camera.regions;
      camera_rects_from_device = true;
      camera_subtract_background = snap.camera.subtract_background;
    }

    if (ImGui::CollapsingHeader("Camera settings")) {
      FlirCameraSettings();
    }
    if (ImGui::CollapsingHeader("Image", ImGuiTreeNodeFlags_DefaultOpen)) {
      FlirImageView();
    }
    if (ImGui::CollapsingHeader("Photometry", ImGuiTreeNodeFlags_DefaultOpen)) {
      FlirPhotometryPanel();
    }

    PushFlirRegionsIfChanged();
    ImGui::End();
  }

  void FlirCameraSettings() {
    ImGui::Text("Commands:");
    ImGui::SameLine();
    for (const std::string &command : snap.camera.device_commands) {
      if (ImGui::Button((command + "##FlirCommand").c_str())) {
        Command([&](Commands &c) {
          c.camera.device_command = command;
          c.camera.device_command_count++;
        });
      }
      ImGui::SameLine();
    }
    ImGui::NewLine();

    static char filename[128] = "";
    ImGui::Text("Filename: %s", snap.camera.filename.c_str());
    ImGui::SameLine();
    ImGui::SetNextItemWidth(400);
    ImGui::InputText("##Filename", filename, sizeof(filename));
    ImGui::SameLine();
    if (ImGui::Button("Set##Filename")) {
      Command([](Commands &c) { c.camera.filename = std::string(filename); });
    }

    static unsigned int n_frames = 1;
    ImGui::Text("Frames:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    ImGui::InputScalar("##NFrames", ImGuiDataType_U32, &n_frames, NULL, NULL, "%d");
    ImGui::SameLine();
    if (ImGui::Button("Record##NFrames")) {
      Command([](Commands &c) {
        c.camera.record_frames = n_frames;
        c.camera.record_count++;
      });
    }

    static unsigned int n_frames_bg = 1;
    ImGui::SameLine();
    ImGui::Text("BG frames:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    ImGui::InputScalar("##NFramesBG", ImGuiDataType_U32, &n_frames_bg, NULL, NULL, "%d");
    ImGui::SameLine();
    if (ImGui::Button("Record BG##NFrames")) {
      Command([](Commands &c) {
        c.camera.background_frames = n_frames_bg;
        c.camera.background_count++;
      });
    }

    SettingRow<double>("Framerate", "%.4f Hz", snap.camera.framerate, ImGuiDataType_Double,
                       [](Commands &c, double v) { c.camera.framerate = v; });
    SettingRow<double>("Integration time", "%.4f ms", snap.camera.integration_time_ms, ImGuiDataType_Double,
                       [](Commands &c, double v) { c.camera.integration_time_ms = v; });
    SettingRow<unsigned int>("Width", "%d px", snap.camera.width, ImGuiDataType_U32,
                             [](Commands &c, unsigned int v) { c.camera.width = v; });
    SettingRow<unsigned int>("Height", "%d px", snap.camera.height, ImGuiDataType_U32,
                             [](Commands &c, unsigned int v) { c.camera.height = v; });
  }

  // Shows the value that the camera reports, and sends a new one when the user asks for it.
  template <typename T, typename Apply>
  void SettingRow(const char *label, const char *format, T reported, ImGuiDataType type, Apply apply) {
    ImGui::Text("%s: ", label);
    ImGui::SameLine();
    ImGui::Text(format, reported);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120);
    static T wanted{};
    ImGui::PushID(label);
    ImGui::InputScalar("##Wanted", type, &wanted, NULL, NULL, format);
    ImGui::SameLine();
    if (ImGui::Button("Set")) {
      const T value = wanted;
      Command([&](Commands &c) { apply(c, value); });
    }
    ImGui::PopID();
  }

  void FlirImageView() {
    static int image_product = 0;
    ImGui::Text("Show:");
    ImGui::SameLine();
    bool product_changed = ImGui::RadioButton("Raw", &image_product, 0);
    ImGui::SameLine();
    product_changed |= ImGui::RadioButton("Background subtracted", &image_product, 1);
    if (product_changed) {
      Command([](Commands &c) { c.camera.image_product = image_product; });
    }

    camera_image = core.blackboard().camera_image.load().value;

    static ImPlotColormap map = ImPlotColormap_Viridis;
    if (ImPlot::ColormapButton(ImPlot::GetColormapName(map), ImVec2(225, 0), map)) {
      map = (map + 1) % ImPlot::GetColormapCount();
      ImPlot::BustColorCache("##Heatmap");
    }

    ImPlot::PushColormap(map);
    ImGui::SameLine();
    static bool autoscale_colormap = true;
    ImGui::Checkbox("Autoscale", &autoscale_colormap);

    if (camera_image.data.empty() || camera_image.width == 0 || camera_image.height == 0) {
      ImPlot::PopColormap();
      ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.0f, 1.0f), "No image. Is the camera streaming?");
      return;
    }

    const unsigned int width = camera_image.width;
    const unsigned int height = camera_image.height;
    int *values = camera_image.data.data();

    const float aspect_ratio = float(width) / float(height);
    const float plot_width = ImGui::GetContentRegionAvail().x - 80 * io->FontGlobalScale;
    const float plot_height = plot_width / aspect_ratio;

    static int scale_min = 0;
    static int scale_max = 16383;
    const int scale_max_default = 16383;
    const int scale_min_default = -16384;

    ImGui::SetNextItemWidth(400);
    ImGui::SliderScalar("Min", ImGuiDataType_S32, &scale_min, &scale_min_default, &scale_max_default, "%d");
    ImGui::SetNextItemWidth(400);
    ImGui::SameLine();
    ImGui::SliderScalar("Max", ImGuiDataType_S32, &scale_max, &scale_min_default, &scale_max_default, "%d");

    if (autoscale_colormap) {
      scale_min = *std::min_element(camera_image.data.begin(), camera_image.data.end());
      scale_max = *std::max_element(camera_image.data.begin(), camera_image.data.end());
    }
    if (scale_min >= scale_max) {
      scale_min = scale_max - 1;
    }

    if (camera_rects.size() != static_cast<size_t>(camera_region_count)) {
      camera_rects.resize(camera_region_count);
    }

    // A first connect puts the regions of the camera on screen.
    if (camera_rects_from_device) {
      camera_rects_from_device = false;
      for (size_t i = 0; i < camera_rects.size() && i < camera_sent_regions.size(); i++) {
        camera_rects[i] = photometry_regions::to_plot_coords(camera_sent_regions[i], width, height);
      }
    }

    if (ImPlot::BeginPlot("##Heatmap", ImVec2(plot_width, plot_height), ImPlotFlags_NoMouseText)) {
      ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_NoDecorations, ImPlotAxisFlags_NoDecorations);
      ImPlot::PlotHeatmap("heat", values, height, width, scale_min, scale_max, nullptr, ImPlotPoint(0, 0),
                          ImPlotPoint(width, height));
      for (int i = 0; i < camera_region_count; i++) {
        if (camera_rects[i].X.Min == camera_rects[i].X.Max && camera_rects[i].Y.Min == camera_rects[i].Y.Max) {
          const double x_offset = (width * 0.05) * i;
          const double y_offset = (height * 0.05) * i;
          camera_rects[i] = {0 + x_offset, double(width) / 2 + x_offset, 0 + y_offset, double(height) / 2 + y_offset};
        }
        const ImVec4 rect_color = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
        ImPlot::DragRect(i, &camera_rects[i].X.Min, &camera_rects[i].Y.Min, &camera_rects[i].X.Max,
                         &camera_rects[i].Y.Max, rect_color, ImPlotDragToolFlags_None);
        ImVec2 pixel_min = ImPlot::PlotToPixels(ImPlotPoint(camera_rects[i].X.Min, camera_rects[i].Y.Min));
        ImVec2 pixel_max = ImPlot::PlotToPixels(ImPlotPoint(camera_rects[i].X.Max, camera_rects[i].Y.Max));
        if (pixel_min.x > pixel_max.x) {
          std::swap(pixel_min.x, pixel_max.x);
        }
        if (pixel_min.y > pixel_max.y) {
          std::swap(pixel_min.y, pixel_max.y);
        }
        ImPlot::PushPlotClipRect();
        ImPlot::GetPlotDrawList()->AddRect(pixel_min, pixel_max, ImGui::ColorConvertFloat4ToU32(rect_color), 0.0f, 0,
                                           2.5f);
        ImPlot::PopPlotClipRect();
        const std::string label = std::to_string(i + 1);
        ImGui::SetWindowFontScale(2.0f);
        ImPlot::PlotText(label.c_str(), camera_rects[i].X.Min, camera_rects[i].Y.Max, ImVec2(24.0f, 24.0f));
        ImGui::SetWindowFontScale(1.0f);
      }
      ImPlot::EndPlot();
    }
    ImGui::SameLine();
    ImPlot::ColormapScale("##HeatScale", scale_min, scale_max, ImVec2(80 * io->FontGlobalScale, plot_height));
    ImPlot::PopColormap();
  }

  // The regions follow the rectangles as they move.
  void PushFlirRegionsIfChanged() {
    if (camera_image.width == 0 || camera_image.height == 0) {
      return;
    }

    std::vector<PhotRegion> regions;
    regions.reserve(camera_region_count);
    for (int i = 0; i < camera_region_count && i < static_cast<int>(camera_rects.size()); i++) {
      regions.push_back(photometry_regions::to_image_coords(camera_rects[i], camera_image.width, camera_image.height));
    }

    bool same = regions.size() == camera_sent_regions.size();
    for (size_t i = 0; i < regions.size() && same; i++) {
      same = regions[i].x0 == camera_sent_regions[i].x0 && regions[i].y0 == camera_sent_regions[i].y0 &&
             regions[i].x1 == camera_sent_regions[i].x1 && regions[i].y1 == camera_sent_regions[i].y1;
    }
    if (same) {
      return;
    }

    Command([&](Commands &c) { c.camera.regions = regions; });
    camera_sent_regions = regions;
  }

  void FlirPhotometryPanel() {
    if (ImGui::Checkbox("Subtract background", &camera_subtract_background)) {
      const bool on = camera_subtract_background;
      Command([on](Commands &c) { c.camera.subtract_background = on; });
    }
    ImGui::SameLine();
    ImGui::Text("Regions:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120);
    if (ImGui::InputInt("##RegionCount", &camera_region_count, 1, 1)) {
      camera_region_count = std::max(0, std::min(camera_region_count, kMaxPhotRegions));
    }

    // Local display scaling. It divides what is drawn and it does not reach the camera.
    static bool apply_intensity_of_one = false;
    static bool apply_nd_filter = false;
    static float nd_filter_factor = 1.0f;
    ImGui::Text("Normalise:");
    ImGui::SameLine();
    ImGui::Checkbox("I_0##Apply one", &apply_intensity_of_one);
    ImGui::SameLine();
    ImGui::Checkbox("ND##Apply ND", &apply_nd_filter);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120);
    ImGui::InputFloat("##ND filter factor", &nd_filter_factor, 0.01f, 0.1f, "%.2f");

    const size_t shown = std::min(static_cast<size_t>(camera_region_count), snap.camera.n_regions);
    for (size_t i = 0; i < shown; i++) {
      double sum = snap.camera.values[i];
      if (apply_intensity_of_one) {
        sum /= phot_intensity_of_one[i];
      }
      if (apply_nd_filter) {
        sum /= nd_filter_factor;
      }
      ImGui::Text("%zu: %.3e", i + 1, sum);
      ImGui::SameLine();
      ImGui::Text("I_0:");
      ImGui::SameLine();
      ImGui::SetNextItemWidth(120);
      ImGui::InputFloat(("##I_0_" + std::to_string(i)).c_str(), &phot_intensity_of_one[i], 0.01f, 0.1f, "%.2f");
    }

    static float history_length = 10.f;
    ImGui::SliderFloat("History", &history_length, 1, 20, "%.5f s", ImGuiSliderFlags_Logarithmic);
    if (ImPlot::BeginPlot("Sum Intensity", ImVec2(-1, 400 * io->FontGlobalScale))) {
      static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
      ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_AutoFit, yflags);
      ImPlot::SetupAxisLimits(ImAxis_X1, phot_time - history_length, phot_time, ImGuiCond_Always);
      ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
      ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
      for (int i = 0; i < camera_region_count; i++) {
        PlotSeries(std::to_string(i + 1).c_str(), phot_buffers[i], i % ImPlot::GetColormapCount(), 2.0f);
      }
      ImPlot::EndPlot();
    }
  }

  bool Initialize() {
    glfwSetErrorCallback(glfw_error_callback);

    // GLFW 3.4 probes Wayland before X11, printing "XDG_RUNTIME_DIR not set"
    // on systems where Wayland is not running. Force X11 directly.
    glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);

    if (!glfwInit()) return false;

#if defined(IMGUI_IMPL_OPENGL_ES2)
    glsl_version = "#version 100";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
    glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#else
    glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#endif

    window = glfwCreateWindow(1280, 720, "NICEcontrol", nullptr, nullptr);
    if (window == nullptr) return false;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);  // Enable vsync

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    io = &ImGui::GetIO();
    io->ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io->ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
    io->ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io->ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

    ImGui::StyleColorsDark();

    // When viewports are enabled we tweak WindowRounding/WindowBg so platform
    // windows can look identical to regular ones.
    ImGuiStyle &style = ImGui::GetStyle();
    if (io->ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
      style.WindowRounding = 5.0f;
      style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    io->Fonts->AddFontDefault();

    ImFontConfig config;
    config.SizePixels = 15.0f * 1.5f;
    config.OversampleH = 1;
    config.OversampleV = 1;
    config.PixelSnapH = true;

    mainFont =
        io->Fonts->AddFontFromMemoryCompressedBase85TTF(SourceSans3Regular_compressed_data_base85, 24.0f, &config);
    if (mainFont == nullptr) return false;

    clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    return true;
  }

  void Cleanup() {
    if (window) {
      ImGui_ImplOpenGL3_Shutdown();
      ImGui_ImplGlfw_Shutdown();
      ImPlot::DestroyContext();
      ImGui::DestroyContext();
      glfwDestroyWindow(window);
      glfwTerminate();
      window = nullptr;
    }
  }

  static void glfw_error_callback(int error, const char *description) {
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
  }
};
