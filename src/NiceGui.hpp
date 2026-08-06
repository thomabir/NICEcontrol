#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <mutex>
#include <numbers>
#include <queue>
#include <random>
#include <thread>

// GUI
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h>

#include "../lib/fonts/SourceSans3Regular.cpp"
#include "../lib/implot/implot.h"

// Data types
#include "ControlData.hpp"
#include "MeasurementT.hpp"
#include "ScrollingBuffer.hpp"
#include "ScrollingBufferT.hpp"
#include "TSCircularBuffer.hpp"

// Controllers (PID etc.)
#include "Controllers.hpp"
#include "DiagPIController.hpp"
#include "MIMOControlLoop.hpp"
#include "SISOControlLoop.hpp"

// Other
#include <algorithm>

#include "Consumer.hpp"
#include "FftCalculator.hpp"          // FFT for data streams
#include "Iir.h"                      // IIR filter from https://github.com/berndporr/iir1
#include "MetrologyReader.hpp"        // reads metrology data and puts into queue
#include "TangoFlirCamInterface.hpp"  // Tango interface for camera
#include "TangoGenericInterface.hpp"  // Tango interface for generic devices
#include "Workers.hpp"

// functions
#include "utils.hpp"

// Windows
#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

class PlcConnection;

class NiceGui {
 public:
  NiceGui(SharedResources &resources, Workers &workers) : res(resources), workers(workers) {}
  ~NiceGui() { Cleanup(); }

  void start() {
    gui_thread = std::jthread([this]() {
      // Note: GLFW is preferably at home in a single thread, splitting it up can lead to issues.
      if (!Initialize()) {
        std::cerr << "Failed to initialize GUI!" << std::endl;
        return;
      }
      // Main GUI loop
      while (!ShouldClose()) {
        RenderFrame();
      }
    });
  }

  void wait_for_close() { gui_thread.join(); }

 private:
  SharedResources &res;
  Workers &workers;
  std::jthread gui_thread;
  GLFWwindow *window = nullptr;
  const char *glsl_version = nullptr;
  ImGuiIO *io = nullptr;
  ImFont *mainFont = nullptr;
  ImVec4 clear_color;
  float t_gui = 0;
  PlcConnection *plc_cached = nullptr;

  // The FLIR camera and the state that its three panels share.
  TangoFlirCamInterface flir_cam;
  bool flir_regions_read_once = false;

  // The regions last sent to the camera.
  std::vector<PhotRegion> flir_sent_regions;
  bool flir_subtract_bg_photometry = false;

  // The rectangles on screen. They go to the camera in each frame in which they differ from flir_sent_regions.
  int flir_region_count = 1;
  std::vector<ImPlotRect> flir_rects;
  bool flir_rects_from_camera = false;
  unsigned int flir_image_width = 0;
  unsigned int flir_image_height = 0;

  // 0 shows the raw image, 1 shows the background subtracted image. Both come from the camera.
  int flir_image_product = 0;

  // Photometry history, one buffer for each possible region.
  // The buffer span sets the upper limit of the history slider. 20000 points give 20 s at 1 kHz.
  static constexpr int kFlirPhotHistoryPoints = 20000;
  std::vector<ScrollingBufferT<double, double>> flir_phot_buffers;
  std::vector<float> flir_intensity_of_one;
  uint64_t flir_last_frame_id = 0;
  std::array<double, kMaxPhotRegions> flir_last_values{};
  size_t flir_last_n_regions = 0;

  bool ShouldClose() const { return glfwWindowShouldClose(window); }

  PlcConnection &plc() {
    if (!plc_cached) {
      plc_cached = &res.ethercat_ads.plc();
    }
    return *plc_cached;
  }

  void RenderFrame() {
    glfwPollEvents();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::PushFont(mainFont);
    RenderUI();
    ImGui::PopFont();

    // Rendering
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w,
                 clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    // Update and Render additional Platform Windows
    if (io->ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
      GLFWwindow *backup_current_context = glfwGetCurrentContext();
      ImGui::UpdatePlatformWindows();
      ImGui::RenderPlatformWindowsDefault();
      glfwMakeContextCurrent(backup_current_context);
    }

    glfwSwapBuffers(window);
  }

  bool Initialize() {
    // Set GLFW error callback
    glfwSetErrorCallback(glfw_error_callback);

    // GLFW 3.4 probes Wayland before X11, printing "XDG_RUNTIME_DIR not set"
    // on systems where Wayland is not running. Force X11 directly.
    glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);

    // Setup window
    if (!glfwInit()) return false;

      // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
    // GL ES 2.0 + GLSL 100
    glsl_version = "#version 100";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
    // GL 3.2 + GLSL 150
    glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
    // GL 3.0 + GLSL 130
    glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#endif

    // Create window with graphics context
    window = glfwCreateWindow(1280, 720, "NICEcontrol", nullptr, nullptr);
    if (window == nullptr) return false;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);  // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    io = &ImGui::GetIO();
    io->ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io->ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
    io->ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io->ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // When viewports are enabled we tweak WindowRounding/WindowBg so platform
    // windows can look identical to regular ones.
    ImGuiStyle &style = ImGui::GetStyle();
    if (io->ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
      style.WindowRounding = 5.0f;
      style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Load fonts
    io->Fonts->AddFontDefault();

    // Configure font
    ImFontConfig config;
    config.SizePixels = 15.0f * 1.5f;
    config.OversampleH = 1;
    config.OversampleV = 1;
    config.PixelSnapH = true;

    // Load Sans font
    mainFont =
        io->Fonts->AddFontFromMemoryCompressedBase85TTF(SourceSans3Regular_compressed_data_base85, 24.0f, &config);

    if (mainFont == nullptr) return false;

    clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    return true;
  }

  void RenderUI() {
    static bool use_work_area = true;
    static ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings;

    // Use the full window for the main NICEcontrol window
    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(use_work_area ? viewport->WorkPos : viewport->Pos);
    ImGui::SetNextWindowSize(use_work_area ? viewport->WorkSize : viewport->Size);

    ImGui::Begin("NICE Control", nullptr, flags);
    ImGuiIO &io = ImGui::GetIO();

    // window management
    static bool show_demo_window = false;
    static bool show_app_metrics = false;

    // buttons for start/stop recording ethercat data
    static bool recording_ethercat = false;
    if (recording_ethercat) {
      if (ImGui::Button("EtherCAT: Stop recording")) {
        recording_ethercat = false;
        // TODO implement recording
        // workers.ethercat_reader.stop_recording();
      }
    } else {
      if (ImGui::Button("EtherCAT: Start recording")) {
        recording_ethercat = true;
        // workers.ethercat_reader.start_recording();
      }
    }

    // Button to start/stop metrology_reader
    static bool metrology_running = true;
    if (metrology_running) {
      if (ImGui::Button("Metrology: Stop")) {
        metrology_running = false;
        workers.metrology_reader.request_stop();
      }
    } else {
      if (ImGui::Button("Metrology: Start")) {
        metrology_running = true;
        workers.metrology_reader.start();
      }
    }

    t_gui = utils::getTime();

    // shutter
    if (ImGui::CollapsingHeader("Shutter")) {
      static TangoGenericInterface shutter("motor/shutter/2");
      shutter.create_device_proxy();
      static std::vector<std::string> command_list = shutter.get_commands();

      // buttons for all found commands
      ImGui::Text("Auto-found commands:");
      ImGui::SameLine();
      for (const auto &command : command_list) {
        if (ImGui::Button(command.c_str())) {
          shutter.run_command(command);
        }
        ImGui::SameLine();
      }
      ImGui::NewLine();
    }

    // ND Filter
    if (ImGui::CollapsingHeader("ND Filter")) {
      static TangoGenericInterface ndfilter("motor/ndfilter/1");
      ndfilter.create_device_proxy();
      static std::vector<std::string> command_list = ndfilter.get_commands();

      // buttons for all found commands
      ImGui::Text("Auto-found commands:");
      ImGui::SameLine();
      for (const auto &command : command_list) {
        if (ImGui::Button(command.c_str())) {
          ndfilter.run_command(command);
        }
        ImGui::SameLine();
      }
      ImGui::NewLine();
    }

    // science camera
    if (ImGui::CollapsingHeader("Flir Camera")) {
      WindowFlirCam();
    }

    // ADC measurements
    if (ImGui::CollapsingHeader("ADC Measurements")) {
      static ScrollingBufferT<int, int> adc_buffers[16];
      static float t_adc = 0;

      // get lates time in ADC queue

      if (!res.metrology.adc_queues[0].isempty()) {
        t_adc = res.metrology.adc_queues[0].back().time;
      }

      // add all measurements to the plot buffers
      for (int i = 0; i < 16; i++) {
        if (!res.metrology.adc_queues[i].isempty()) {
          int N = res.metrology.adc_queues[i].size();
          for (int j = 0; j < N; j++) {
            auto m = res.metrology.adc_queues[i].pop();
            adc_buffers[i].AddPoint(m.time, m.value);
          }
        }
      }

      static float adc_history_length = 1000.f;
      ImGui::SliderFloat("ADC History", &adc_history_length, 200, 70000, "%.0f samples", ImGuiSliderFlags_Logarithmic);

      // plot label names
      static const char *plot_labels[16] = {
          "QOD1 UP",        "QPD1 LEFT", "QPD1 RIGHT",     "QPD1 DOWN", "QPD1 sum", "OPD ref",
          "Pos ref (QPD1)", "OPD back",  "Pos ref (QPD2)", "NC3",       "NC4",      "NC5",
          "QPD2 UP",        "QPD2 LEFT", "QPD2 RIGHT",     "QPD2 DOWN"};

      // plot style
      static float thickness = 2;
      static ImPlotAxisFlags xflags = ImPlotAxisFlags_NoTickLabels;
      static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;

      if (ImPlot::BeginPlot("##ADC", ImVec2(-1, 400 * io.FontGlobalScale))) {
        ImPlot::SetupAxes(nullptr, nullptr, xflags, yflags);
        ImPlot::SetupAxisLimits(ImAxis_X1, t_adc - adc_history_length, t_adc, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
        for (int i = 0; i < 16; i++) {
          ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(i), thickness);
          ImPlot::PlotLine(plot_labels[i], &adc_buffers[i].Data[0].time, &adc_buffers[i].Data[0].value,
                           adc_buffers[i].Data.size(), 0, adc_buffers[i].Offset, 2 * sizeof(int));
        }
        ImPlot::EndPlot();
      }
    }

    // EtherCAT monitor
    if (ImGui::CollapsingHeader("EtherCAT Monitor")) {
      WindowEtheratMonitor();
    }

    if (ImGui::CollapsingHeader("OPD")) {
      WindowOPD();
    }

    if (ImGui::CollapsingHeader("Lateral beam control")) {
      WindowBeamControl();
    }

    if (ImGui::CollapsingHeader("Program settings")) {
      ImGui::DragFloat("GUI scale", &io.FontGlobalScale, 0.005f, 0.5, 6.0, "%.2f",
                       ImGuiSliderFlags_AlwaysClamp);  // Scale everything
      // button to show demo windows
      ImGui::Checkbox("Show demo windows", &show_demo_window);
      ImGui::Checkbox("Show app metrics", &show_app_metrics);
    }
    ImGui::End();

    // demo window
    if (show_demo_window) {
      ImGui::ShowDemoWindow();
      ImPlot::ShowDemoWindow();
    }
    if (show_app_metrics) {
      // Show app metrics
      ImGui::ShowMetricsWindow();
    }
  }

  struct OpdSettings {
    float setpoint_um = 0.0f;
    float open_loop_cmd_um = 0.0f;
    float kp = 0.0f;
    float ki = 1.0f;
    bool reset_phase = false;
    int mode = 0;  // 0: do nothing, 1: direct control of delay line, 3: closed loop. 2 is reserved for a specific
                   // open-loop mode.
  };

  void WindowOPD() {
    if (ImGui::TreeNode("Control##OPD")) {
      static OpdSettings current;

      bool mode_changed = false;
      ImGui::Text("Control mode:");
      ImGui::SameLine();
      mode_changed |= ImGui::RadioButton("Off##OPD", &current.mode, 0);
      ImGui::SameLine();
      mode_changed |= ImGui::RadioButton("Open loop##OPD", &current.mode, 1);
      ImGui::SameLine();
      mode_changed |= ImGui::RadioButton("Closed loop##OPD", &current.mode, 3);
      if (mode_changed) {
        plc().write<int16_t>("MAIN.opd_mode", static_cast<int16_t>(current.mode));
        std::cout << "Changed control mode to " << current.mode << "\n";
      }

      // Setpoint
      const float opd_setpoint_min = -1e3, opd_setpoint_max = 1e3;
      if (ImGui::DragFloat("OPD Setpoint", &current.setpoint_um, 1e-4, opd_setpoint_min, opd_setpoint_max, "%.4f um",
                           ImGuiSliderFlags_AlwaysClamp)) {
        plc().write<float>("MAIN.opd_setpoint_um", current.setpoint_um);
      }

      // slider for Open loop position command (clamp 0 to 15 um)
      const float open_loop_min = 0.0f, open_loop_max = 15.0f;
      if (ImGui::DragFloat("Open loop DL command", &current.open_loop_cmd_um, 1e-4, open_loop_min, open_loop_max,
                           "%.4f um", ImGuiSliderFlags_AlwaysClamp)) {
        plc().write<float>("MAIN.opd_open_loop_cmd_um", current.open_loop_cmd_um);
      }

      // P and I control loop gains
      if (ImGui::SliderFloat("P##OPD EtherCAT", &current.kp, 1e-4f, 1e0f, "%.5f", ImGuiSliderFlags_Logarithmic)) {
        plc().write<float>("MAIN.opd_kp", current.kp);
      }
      if (ImGui::SliderFloat("I##OPD EtherCAT", &current.ki, 1e-1f, 1e3f, "%.5f", ImGuiSliderFlags_Logarithmic)) {
        plc().write<float>("MAIN.opd_ki", current.ki);
      }

      // Reset phase unwrap
      if (ImGui::Button("Reset phase unwrap")) {
        plc().write<bool>("MAIN.reset_unwrap", true);
        plc().write<bool>("MAIN.reset_unwrap", false);
      }

      // old, delete:
      // Interface to EtherCAT master: Send a few bytes via UDP to the master receiver
      // 5 bytes data via UDP:
      // 4 bytes: OPD setpoint float
      // 1 byte: flags: X X X X X X reset_phase run_control_loop
      // const int udp_port = 8888;
      // const char *udp_ip = "192.168.88.177";
      // static EthercatUdpInterface ec_udp_if(udp_ip, udp_port);
      // ec_udp_if.send_commands(current.setpoint_um * 1.0e-3, current.kp, current.ki, current.reset_phase,
      //                         current.mode == 2);

      ImGui::TreePop();
    }
  }

  void WindowBeamControl() {
    // control mode selector
    // raw actuator commands, open loop, closed loop
    static int tip_tilt_loop_select = 0;
    ImGui::Text("Control mode:");
    ImGui::SameLine();
    ImGui::RadioButton("Open loop##TipTilt", &tip_tilt_loop_select, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Closed loop##TipTilt", &tip_tilt_loop_select, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Hold still", &tip_tilt_loop_select, 2);

    // raw actuator commands
    static float tip_tilt_raw_x1 = 0.0f;
    static float tip_tilt_raw_y1 = 0.0f;
    static float tip_tilt_raw_x2 = 0.0f;
    static float tip_tilt_raw_y2 = 0.0f;

    // sliders to set raw actuator commands
    ImGui::Text("Open loop commands:");
    ImGui::DragFloat("X1##TipTiltRaw", &tip_tilt_raw_x1, 0.01f, -1000.0f, 1000.0f, "%.2f urad",
                     ImGuiSliderFlags_AlwaysClamp);
    ImGui::SameLine();
    ImGui::Text("(Encoder: %.0f)", res.piezos.tt1.readx());
    ImGui::DragFloat("Y1##TipTiltRaw", &tip_tilt_raw_y1, 0.01f, -1000.0f, 1000.0f, "%.2f urad",
                     ImGuiSliderFlags_AlwaysClamp);
    ImGui::SameLine();
    ImGui::Text("(Encoder: %.0f)", res.piezos.tt1.ready());
    ImGui::DragFloat("X2##TipTiltRaw", &tip_tilt_raw_x2, 0.01f, -1000.0f, 1000.0f, "%.2f urad",
                     ImGuiSliderFlags_AlwaysClamp);
    ImGui::SameLine();
    ImGui::Text("(Encoder: %.0f)", res.piezos.tt2.readx());
    ImGui::DragFloat("Y2##TipTiltRaw", &tip_tilt_raw_y2, 0.01f, -1000.0f, 1000.0f, "%.2f urad",
                     ImGuiSliderFlags_AlwaysClamp);
    ImGui::SameLine();
    ImGui::Text("(Encoder: %.0f)", res.piezos.tt2.ready());

    workers.beam_controller.set_shear_loop_select(tip_tilt_loop_select);
    workers.beam_controller.move_to_x1(tip_tilt_raw_x1);
    workers.beam_controller.move_to_y1(tip_tilt_raw_y1);
    workers.beam_controller.move_to_x2(tip_tilt_raw_x2);
    workers.beam_controller.move_to_y2(tip_tilt_raw_y2);
  }

  void WindowEtheratMonitor() {
    static ScrollingBufferT<double, double> dl_meas_buffer;
    static ScrollingBufferT<double, double> dl_cmd_buffer;
    static ScrollingBufferT<double, double> metr_opd_um_buffer;
    static ScrollingBufferT<double, double> metr_qpd_buffer[12];
    static ScrollingBufferT<double, double> metr_pointing_buffer[4];
    static double t_ecat = 0;

    // Get data via ADS
    static auto ads_consumer = res.ethercat_ads.data.subscribe();
    PlcSample plc_sample;
    while (res.ethercat_ads.data.try_pop(ads_consumer, plc_sample)) {
      t_ecat = plc_sample.timestamp_ns * 1e-9;  // convert nanoseconds to seconds
      dl_meas_buffer.AddPoint(t_ecat, plc_sample.dl_pos_um);
      dl_cmd_buffer.AddPoint(t_ecat, plc_sample.dl_cmd_um);
      metr_opd_um_buffer.AddPoint(t_ecat, plc_sample.opd_um);  // convert um to nm
    }

    // plot the data
    if (ImGui::TreeNode("Delay line##EtherCAT Monitor Delay Line")) {
      static float history_length = 10.0f;
      ImGui::SliderFloat("History length", &history_length, 0.1f, 10.0f, "%.2f s", ImGuiSliderFlags_Logarithmic);
      static ImPlotAxisFlags xflags = ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoTickLabels;
      static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
      static float thickness = 3.0f * io->FontGlobalScale;
      if (ImPlot::BeginPlot("##EtherCAT Monitor", ImVec2(-1, 200 * io->FontGlobalScale))) {
        ImPlot::SetupAxes(nullptr, nullptr, xflags, yflags);
        ImPlot::SetupAxisLimits(ImAxis_X1, t_ecat - history_length, t_ecat, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(0), thickness);
        ImPlot::PlotLine("DL position (um)", &dl_meas_buffer.Data[0].time, &dl_meas_buffer.Data[0].value,
                         dl_meas_buffer.Data.size(), 0, dl_meas_buffer.Offset, 2 * sizeof(double));
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(1), thickness);
        ImPlot::PlotLine("DL command (um)", &dl_cmd_buffer.Data[0].time, &dl_cmd_buffer.Data[0].value,
                         dl_cmd_buffer.Data.size(), 0, dl_cmd_buffer.Offset, 2 * sizeof(double));
        ImPlot::EndPlot();
      }
      ImGui::TreePop();
    }

    if (ImGui::TreeNode("OPD##EtherCAT Monitor OPD")) {
      static float history_length = 10.0f;
      ImGui::SliderFloat("History length", &history_length, 0.1f, 10.0f, "%.2f s", ImGuiSliderFlags_Logarithmic);
      static ImPlotAxisFlags xflags = ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoTickLabels;
      static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
      static float thickness = 1.0f * io->FontGlobalScale;
      if (ImPlot::BeginPlot("##EtherCAT Monitor Metrology OPD", ImVec2(-1, 200 * io->FontGlobalScale))) {
        ImPlot::SetupAxes(nullptr, nullptr, xflags, yflags);
        ImPlot::SetupAxisLimits(ImAxis_X1, t_ecat - history_length, t_ecat, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -1e6, 1e6);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(1), thickness);
        ImPlot::PlotLine("OPD (um)", &metr_opd_um_buffer.Data[0].time, &metr_opd_um_buffer.Data[0].value,
                         metr_opd_um_buffer.Data.size(), 0, metr_opd_um_buffer.Offset, 2 * sizeof(double));
        ImPlot::EndPlot();

        static float mean = 0.0f;
        static float stddev = 0.0f;
        const int N_points_stats = 1000;

        if (metr_opd_um_buffer.Data.size() > N_points_stats + 2) {
          // get the last 1000 measurements using ImVector<ImVec2> GetLastN(int n)
          auto last_1000_measurements_data = metr_opd_um_buffer.GetLastN(N_points_stats);
          // get the y values
          std::vector<float> last_1000_measurements;
          for (auto &m : last_1000_measurements_data) {
            last_1000_measurements.push_back(m.value);
          }

          // calculate mean and stdev
          mean = std::accumulate(last_1000_measurements.begin(), last_1000_measurements.end(), 0.0) /
                 last_1000_measurements.size();
          float sq_sum = std::inner_product(last_1000_measurements.begin(), last_1000_measurements.end(),
                                            last_1000_measurements.begin(), 0.0);
          stddev = std::sqrt(sq_sum / last_1000_measurements.size() - mean * mean);
        }

        // Display mean and std
        ImGui::Text("Last %d samples: Mean: %.4f um, Std: %.4f nm", N_points_stats, mean, stddev * 1e3);
      }
      ImGui::TreePop();
    }

    if (ImGui::TreeNode("OPD FFT##OPD")) {
      // set up fft
      const static int fs = 1 / 150e-6;      // sampling frequency, Hz (150 us per sample)
      const static int fft_size = 1024 * 8;  // 1.2 s of data
      static double fft_power[fft_size / 2];
      static double fft_freq[fft_size / 2];
      static FFT_calculator<double, double> fft(fft_size, fs, &metr_opd_um_buffer, fft_power, fft_freq);

      fft.calculate();

      // Plot amplitude spectral density of OPD
      static float fft_thickness = 3;
      static ImPlotAxisFlags fft_xflags = ImPlotAxisFlags_None;
      static ImPlotAxisFlags fft_yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
      if (ImPlot::BeginPlot("##FFT", ImVec2(-1, 400 * io->FontGlobalScale))) {
        ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
        ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
        ImPlot::SetupAxes(nullptr, nullptr, fft_xflags, fft_yflags);
        ImPlot::SetupAxisLimits(ImAxis_X1, 1, 3333);
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(1), fft_thickness);
        ImPlot::PlotLine("OPD PSD (nm/sqrtHz)", &fft_freq[0], &fft_power[0], fft_size / 2);
        ImPlot::EndPlot();
      }

      ImGui::TreePop();
    }

    if (ImGui::TreeNode("QPD##EtherCAT Monitor")) {
      static const char *plot_labels[12] = {"QPD1 X1", "QPD1 Y1", "QPD1 I1", "QPD1 X2", "QPD1 Y2", "QPD1 I2",
                                            "QPD2 X1", "QPD2 Y1", "QPD2 I1", "QPD2 X2", "QPD2 Y2", "QPD2 I2"};
      static float history_length = 10.0f;
      ImGui::SliderFloat("History length", &history_length, 0.1f, 10.0f, "%.2f s", ImGuiSliderFlags_Logarithmic);
      static ImPlotAxisFlags xflags = ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoTickLabels;
      static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
      static float thickness = 1.0f * io->FontGlobalScale;
      if (ImPlot::BeginPlot("##EtherCAT Monitor Metrology QPD1", ImVec2(-1, 200 * io->FontGlobalScale))) {
        ImPlot::SetupAxes(nullptr, nullptr, xflags, yflags);
        ImPlot::SetupAxisLimits(ImAxis_X1, t_ecat - history_length, t_ecat, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -1e6, 1e6);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
        for (int i = 0; i < 12; i++) {
          ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(i), thickness);
          ImPlot::PlotLine(plot_labels[i], &metr_qpd_buffer[i].Data[0].time, &metr_qpd_buffer[i].Data[0].value,
                           metr_qpd_buffer[i].Data.size(), 0, metr_qpd_buffer[i].Offset, 2 * sizeof(double));
        }
        ImPlot::EndPlot();
      }

      ImGui::TreePop();
    }

    if (ImGui::TreeNode("Pointing##EtherCAT Monitor")) {
      static const char *plot_labels[4] = {"Beam 1 X", "Beam 1 Y", "Beam 2 X", "Beam 2 Y"};
      static float history_length = 10.0f;
      ImGui::SliderFloat("History length", &history_length, 0.1f, 10.0f, "%.2f s", ImGuiSliderFlags_Logarithmic);
      static ImPlotAxisFlags xflags = ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoTickLabels;
      static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
      static float thickness = 1.0f * io->FontGlobalScale;
      if (ImPlot::BeginPlot("##EtherCAT Monitor Metrology Pointing", ImVec2(-1, 200 * io->FontGlobalScale))) {
        ImPlot::SetupAxes(nullptr, nullptr, xflags, yflags);
        ImPlot::SetupAxisLimits(ImAxis_X1, t_ecat - history_length, t_ecat, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -1e6, 1e6);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
        for (int i = 0; i < 4; i++) {
          ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(i), thickness);
          ImPlot::PlotLine(plot_labels[i], &metr_pointing_buffer[i].Data[0].time,
                           &metr_pointing_buffer[i].Data[0].value, metr_pointing_buffer[i].Data.size(), 0,
                           metr_pointing_buffer[i].Offset, 2 * sizeof(double));
        }
        ImPlot::EndPlot();
      }

      ImGui::TreePop();
    }
  }

  // The FLIR camera window, in three parts: Camera settings, Image, Photometry.
  // The camera computes the photometry. This GUI sets the regions and draws what comes back.
  void WindowFlirCam() {
    static ImVec2 window_size(600, 800);
    ImGui::SetNextWindowSize(window_size, ImGuiCond_FirstUseEver);
    ImGui::Begin("FlirCamWindow", nullptr, 0);

    if (!flir_cam.is_connected()) {
      if (ImGui::Button("Connect")) {
        flir_cam.connect();
      }
      ImGui::SameLine();
      ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "Disconnected");
      ImGui::End();
      return;
    }

    if (ImGui::Button("Disconnect")) {
      flir_cam.disconnect();
    }
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Connected");
    ImGui::SameLine();
    if (ImGui::Button("Ping")) {
      flir_cam.ping_device();
    }

    // Take the settings of the camera once.
    if (!flir_regions_read_once) {
      flir_regions_read_once = true;
      flir_subtract_bg_photometry = flir_cam.get_phot_subtract_background();
      const std::vector<PhotRegion> regions = flir_cam.get_regions();
      if (!regions.empty()) {
        flir_region_count = static_cast<int>(regions.size());
        flir_sent_regions = regions;
        flir_rects_from_camera = true;
      }
    }

    // Poll outside the panels. A collapsed Photometry section keeps the history complete.
    PollFlirPhotometry();

    if (ImGui::CollapsingHeader("Camera settings")) {
      FlirCameraSettings();
    }
    if (ImGui::CollapsingHeader("Image", ImGuiTreeNodeFlags_DefaultOpen)) {
      FlirImageView();
    }
    if (ImGui::CollapsingHeader("Photometry", ImGuiTreeNodeFlags_DefaultOpen)) {
      FlirPhotometryPanel();
    }

    // The regions follow the rectangles as they move.
    PushFlirRegionsIfChanged();

    ImGui::End();
  }

  void FlirCameraSettings() {
    static std::vector<std::string> command_list = flir_cam.get_commands();

    ImGui::Text("Commands:");
    ImGui::SameLine();
    for (const auto &command : command_list) {
      if (ImGui::Button(command.c_str())) {
        flir_cam.run_command(command);
      }
      ImGui::SameLine();
    }
    ImGui::NewLine();

    static char filename_char[128] = "";
    static std::string filename_str = flir_cam.get_filename();
    ImGui::Text("Filename:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(500);
    ImGui::InputText("##Filename", filename_char, sizeof(filename_char));
    ImGui::SameLine();
    if (ImGui::Button("Set##Filename")) {
      filename_str = std::string(filename_char);
      flir_cam.set_filename(filename_str);
    }
    ImGui::SameLine();
    if (ImGui::Button("Read##Filename")) {
      filename_str = flir_cam.get_filename();
      std::strncpy(filename_char, filename_str.c_str(), sizeof(filename_char) - 1);
      filename_char[sizeof(filename_char) - 1] = '\0';
    }

    static unsigned int n_frames = 1;
    ImGui::SameLine();
    ImGui::Text("Frames:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    ImGui::InputScalar("##NFramesSlider", ImGuiDataType_U32, &n_frames, NULL, NULL, "%d");
    ImGui::SameLine();
    if (ImGui::Button("Record##NFrames")) {
      flir_cam.start_recording(n_frames);
    }

    static unsigned int n_frames_bg = 1;
    ImGui::Text("BG frames:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    ImGui::InputScalar("##NFramesBGSlider", ImGuiDataType_U32, &n_frames_bg, NULL, NULL, "%d");
    ImGui::SameLine();
    if (ImGui::Button("Record BG##NFrames")) {
      flir_cam.start_recording_background(n_frames_bg);
    }

    static double framerate = flir_cam.read_framerate();
    ImGui::Text("Framerate:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    ImGui::InputScalar("##FramerateSlider", ImGuiDataType_Double, &framerate, NULL, NULL, "%.4f Hz");
    ImGui::SameLine();
    if (ImGui::Button("Read##Framerate")) {
      framerate = flir_cam.read_framerate();
    }
    ImGui::SameLine();
    if (ImGui::Button("Set##Framerate")) {
      flir_cam.write_framerate(framerate);
      framerate = flir_cam.read_framerate();
    }

    static double integration_time = flir_cam.read_integration_time();
    ImGui::SameLine();
    ImGui::Text("Integration time:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    ImGui::InputScalar("##IntegrationTimeSlider", ImGuiDataType_Double, &integration_time, NULL, NULL, "%.4f ms");
    ImGui::SameLine();
    if (ImGui::Button("Read##IntegrationTime")) {
      integration_time = flir_cam.read_integration_time();
    }
    ImGui::SameLine();
    if (ImGui::Button("Set##IntegrationTime")) {
      flir_cam.write_integration_time(integration_time);
      integration_time = flir_cam.read_integration_time();
    }

    static unsigned int gui_width = flir_cam.get_width();
    ImGui::SameLine();
    ImGui::Text("Width:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    ImGui::InputScalar("##WidthSlider", ImGuiDataType_S32, &gui_width, NULL, NULL, "%d px");
    ImGui::SameLine();
    if (ImGui::Button("Read##Width")) {
      gui_width = flir_cam.get_width();
    }
    ImGui::SameLine();
    if (ImGui::Button("Set##Width")) {
      flir_cam.write_width(gui_width);
    }

    static unsigned int gui_height = flir_cam.get_height();
    ImGui::SameLine();
    ImGui::Text("Height:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    ImGui::InputScalar("##HeightSlider", ImGuiDataType_S32, &gui_height, NULL, NULL, "%d px");
    ImGui::SameLine();
    if (ImGui::Button("Read##Height")) {
      gui_height = flir_cam.get_height();
    }
    ImGui::SameLine();
    if (ImGui::Button("Set##Height")) {
      flir_cam.write_height(gui_height);
    }
  }

  void FlirImageView() {
    // The camera publishes both products. This choice selects which one to draw.
    ImGui::Text("Show:");
    ImGui::SameLine();
    ImGui::RadioButton("Raw", &flir_image_product, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Background subtracted", &flir_image_product, 1);

    static Image<int> image;
    image = (flir_image_product == 1) ? flir_cam.get_image_bg_sub() : flir_cam.get_image();

    static ImPlotColormap map = ImPlotColormap_Viridis;
    if (ImPlot::ColormapButton(ImPlot::GetColormapName(map), ImVec2(225, 0), map)) {
      map = (map + 1) % ImPlot::GetColormapCount();
      // We bust the color cache of our plots so that item colors will
      // resample the new colormap in the event that they have already
      // been created. See documentation in implot.h.
      ImPlot::BustColorCache("##Heatmap1");
      ImPlot::BustColorCache("##Heatmap2");
    }

    ImPlot::PushColormap(map);
    ImGui::SameLine();
    static bool autoscale_colormap = true;
    ImGui::Checkbox("Autoscale", &autoscale_colormap);

    if (image.data.empty() || image.width == 0 || image.height == 0) {
      ImPlot::PopColormap();
      ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.0f, 1.0f), "No image. Is the camera streaming?");
      return;
    }
    flir_image_width = image.width;
    flir_image_height = image.height;

    int *values = image.data.data();

    float aspect_ratio = float(image.width) / float(image.height);
    float plot_width = ImGui::GetContentRegionAvail().x - 80 * io->FontGlobalScale;  // leave space for colormap
    float plot_height = plot_width / aspect_ratio;

    int img_min_count = *std::min_element(image.data.begin(), image.data.end());
    int img_max_count = *std::max_element(image.data.begin(), image.data.end());

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
      scale_min = img_min_count;
      scale_max = img_max_count;
    }
    if (scale_min >= scale_max) {
      scale_min = scale_max - 1;
    }

    ImPlotDragToolFlags flags = ImPlotDragToolFlags_None;
    if (flir_rects.size() != static_cast<size_t>(flir_region_count)) {
      flir_rects.resize(flir_region_count);
    }

    // A first connect puts the regions of the camera on screen.
    if (flir_rects_from_camera) {
      flir_rects_from_camera = false;
      for (size_t i = 0; i < flir_rects.size() && i < flir_sent_regions.size(); i++) {
        flir_rects[i] = photometry_regions::to_plot_coords(flir_sent_regions[i], image.width, image.height);
      }
    }

    if (ImPlot::BeginPlot("##Heatmap2", ImVec2(plot_width, plot_height), ImPlotFlags_NoMouseText)) {
      ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_NoDecorations, ImPlotAxisFlags_NoDecorations);
      ImPlot::PlotHeatmap("heat1", values, image.height, image.width, scale_min, scale_max, nullptr, ImPlotPoint(0, 0),
                          ImPlotPoint(image.width, image.height));
      for (int i = 0; i < flir_region_count; i++) {
        if (flir_rects[i].X.Min == flir_rects[i].X.Max && flir_rects[i].Y.Min == flir_rects[i].Y.Max) {
          const double x_offset = (image.width * 0.05) * i;
          const double y_offset = (image.height * 0.05) * i;
          flir_rects[i] = {0 + x_offset, double(image.width) / 2 + x_offset, 0 + y_offset,
                           double(image.height) / 2 + y_offset};
        }
        bool rect_clicked = false;
        bool rect_hovered = false;
        bool rect_held = false;
        const ImVec4 rect_color = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
        ImPlot::DragRect(i, &flir_rects[i].X.Min, &flir_rects[i].Y.Min, &flir_rects[i].X.Max, &flir_rects[i].Y.Max,
                         ImVec4(rect_color.x, rect_color.y, rect_color.z, 1.0f), flags, &rect_clicked, &rect_hovered,
                         &rect_held);
        const ImPlotPoint rect_min(flir_rects[i].X.Min, flir_rects[i].Y.Min);
        const ImPlotPoint rect_max(flir_rects[i].X.Max, flir_rects[i].Y.Max);
        ImVec2 pixel_min = ImPlot::PlotToPixels(rect_min);
        ImVec2 pixel_max = ImPlot::PlotToPixels(rect_max);
        if (pixel_min.x > pixel_max.x) {
          std::swap(pixel_min.x, pixel_max.x);
        }
        if (pixel_min.y > pixel_max.y) {
          std::swap(pixel_min.y, pixel_max.y);
        }
        const ImU32 border_color =
            ImGui::ColorConvertFloat4ToU32(ImVec4(rect_color.x, rect_color.y, rect_color.z, 1.0f));
        ImPlot::PushPlotClipRect();
        ImPlot::GetPlotDrawList()->AddRect(pixel_min, pixel_max, border_color, 0.0f, 0, 2.5f);
        ImPlot::PopPlotClipRect();
        const double label_x = flir_rects[i].X.Min;
        const double label_y = flir_rects[i].Y.Max;
        const std::string label = std::to_string(i + 1);
        ImGui::SetWindowFontScale(2.0f);
        ImPlot::PlotText(label.c_str(), label_x, label_y, ImVec2(24.0f, 24.0f));
        ImGui::SetWindowFontScale(1.0f);
      }
      ImPlot::EndPlot();
    }
    ImGui::SameLine();
    ImPlot::ColormapScale("##HeatScale", scale_min, scale_max, ImVec2(80 * io->FontGlobalScale, plot_height));
    ImPlot::PopColormap();
  }

  // Send the rectangles to the camera when they differ from the ones last sent.
  void PushFlirRegionsIfChanged() {
    if (flir_image_width == 0 || flir_image_height == 0) {
      return;
    }

    std::vector<PhotRegion> regions;
    regions.reserve(flir_region_count);
    for (int i = 0; i < flir_region_count && i < static_cast<int>(flir_rects.size()); i++) {
      regions.push_back(photometry_regions::to_image_coords(flir_rects[i], flir_image_width, flir_image_height));
    }

    bool same = regions.size() == flir_sent_regions.size();
    for (size_t i = 0; i < regions.size() && same; i++) {
      same = regions[i].x0 == flir_sent_regions[i].x0 && regions[i].y0 == flir_sent_regions[i].y0 &&
             regions[i].x1 == flir_sent_regions[i].x1 && regions[i].y1 == flir_sent_regions[i].y1;
    }
    if (same) {
      return;
    }

    flir_cam.set_regions(regions);
    flir_sent_regions = regions;
  }

  // Take every sample after the last frame that this GUI holds.
  void PollFlirPhotometry() {
    if (flir_phot_buffers.size() != static_cast<size_t>(kMaxPhotRegions)) {
      // One buffer for each possible region. A change of the region count keeps the history of the others.
      flir_phot_buffers.resize(kMaxPhotRegions, ScrollingBufferT<double, double>(kFlirPhotHistoryPoints));
      flir_intensity_of_one.resize(kMaxPhotRegions, 1.0f);
    }

    const PhotBatch batch = flir_cam.get_phot_since(flir_last_frame_id);
    if (batch.samples.empty()) {
      return;
    }
    if (flir_last_frame_id != 0 && batch.samples.front().frame_id > flir_last_frame_id + 1) {
      std::cerr << "FlirCam: photometry gap, frames " << (flir_last_frame_id + 1) << " to "
                << (batch.samples.front().frame_id - 1) << " were lost." << std::endl;
    }
    flir_last_frame_id = batch.samples.back().frame_id;

    // The samples carry camera time, the plots use GUI time. The newest sample takes the current GUI time and the
    // others follow at their camera time.
    const double newest_camera_time = batch.samples.back().timestamp_s;
    for (const auto &sample : batch.samples) {
      const double t = t_gui - (newest_camera_time - sample.timestamp_s);
      for (size_t r = 0; r < batch.n_regions && r < static_cast<size_t>(kMaxPhotRegions); r++) {
        flir_phot_buffers[r].AddPoint(t, sample.values[r]);
      }
    }
    flir_last_values = batch.samples.back().values;
    flir_last_n_regions = batch.n_regions;
  }

  void FlirPhotometryPanel() {
    if (ImGui::Checkbox("Subtract background", &flir_subtract_bg_photometry)) {
      flir_cam.set_phot_subtract_background(flir_subtract_bg_photometry);
    }
    ImGui::SameLine();
    ImGui::Text("Regions:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120);
    if (ImGui::InputInt("##PhotometryRectCount", &flir_region_count, 1, 1)) {
      flir_region_count = std::max(0, std::min(flir_region_count, kMaxPhotRegions));
    }

    // Local display scaling. It divides what is drawn.
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

    // One row for each region that the camera reported. A row shows the last count that it had.
    const size_t shown = std::min(static_cast<size_t>(flir_region_count), flir_last_n_regions);
    for (size_t i = 0; i < shown; i++) {
      double sum = flir_last_values[i];
      if (apply_intensity_of_one) {
        sum /= flir_intensity_of_one[i];
      }
      if (apply_nd_filter) {
        sum /= nd_filter_factor;
      }
      ImGui::Text("%zu: %.3e", i + 1, sum);
      ImGui::SameLine();
      ImGui::Text("I_0:");
      ImGui::SameLine();
      ImGui::SetNextItemWidth(120);
      ImGui::InputFloat(("##I_0_" + std::to_string(i)).c_str(), &flir_intensity_of_one[i], 0.01f, 0.1f, "%.2f");
    }

    static float mean_intensity_history_length = 10.f;
    // The upper limit is the span that the buffers hold at the full camera rate.
    ImGui::SliderFloat("History", &mean_intensity_history_length, 1, 20, "%.5f s", ImGuiSliderFlags_Logarithmic);
    if (ImPlot::BeginPlot("Sum Intensity", ImVec2(-1, 400 * io->FontGlobalScale))) {
      static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
      ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_AutoFit, yflags);
      ImPlot::SetupAxisLimits(ImAxis_X1, t_gui - mean_intensity_history_length, t_gui, ImGuiCond_Always);
      ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
      ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
      for (int i = 0; i < flir_region_count; i++) {
        if (flir_phot_buffers[i].Data.empty()) {
          continue;
        }
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(i % ImPlot::GetColormapCount()), 2);
        const std::string label = std::to_string(i + 1);
        ImPlot::PlotLine(label.c_str(), &flir_phot_buffers[i].Data[0].time, &flir_phot_buffers[i].Data[0].value,
                         flir_phot_buffers[i].Data.size(), 0, flir_phot_buffers[i].Offset, 2 * sizeof(double));
      }
      ImPlot::EndPlot();
    }
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

  // GLFW error callback
  static void glfw_error_callback(int error, const char *description) {
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
  }
};
