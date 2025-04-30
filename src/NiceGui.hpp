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
#include "EthercatUdpInterface.hpp"  // EtherCAT UDP Interface
#include "FftCalculator.hpp"         // FFT for data streams
#include "Iir.h"                     // IIR filter from https://github.com/berndporr/iir1
#include "MetrologyReader.hpp"       // reads metrology data and puts into queue
#include "Workers.hpp"
#include "camera_if.hpp"  // Tango interface for camera

// functions
#include "characterise_control_loop.hpp"
#include "characterise_joint_closed_loop.hpp"
#include "utils.hpp"

// Windows
#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

class NiceGui {
 public:
  NiceGui() = default;
  ~NiceGui() { Cleanup(); }

  bool Initialize() {
    // Set GLFW error callback
    glfwSetErrorCallback(glfw_error_callback);

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

  bool ShouldClose() const { return glfwWindowShouldClose(window); }

  void StartFrame() {
    glfwPollEvents();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
  }

  void RenderFrame(SharedResources &resources, Workers &workers) {
    ImGui::PushFont(mainFont);
    RenderUI(resources, workers);
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

 private:
  GLFWwindow *window = nullptr;
  const char *glsl_version = nullptr;
  ImGuiIO *io = nullptr;
  ImFont *mainFont = nullptr;
  ImVec4 clear_color;

  void RenderUI(SharedResources &res, Workers &workers) {
    static bool use_work_area = true;
    static ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings;

    // Use the full window for the main NICEcontrol window
    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(use_work_area ? viewport->WorkPos : viewport->Pos);
    ImGui::SetNextWindowSize(use_work_area ? viewport->WorkSize : viewport->Size);

    ImGui::Begin("NICE Control", nullptr, flags);
    ImGuiIO &io = ImGui::GetIO();

    // shear control
    float shear_x1_setpoint_gui = 0.0f;
    float shear_x2_setpoint_gui = 0.0f;
    float shear_y1_setpoint_gui = 0.0f;
    float shear_y2_setpoint_gui = 0.0f;

    // pointing control
    float pointing_x1_setpoint_gui = 0.0f;
    float pointing_x2_setpoint_gui = 0.0f;
    float pointing_y1_setpoint_gui = 0.0f;
    float pointing_y2_setpoint_gui = 0.0f;

    // shear control <-> GUI
    static int shear_loop_select = 0;
    static float shear_p_gui = 0.4f;
    static float shear_i_gui = 0.007f;

    // pointing control <-> GUI
    static int pointing_loop_select = 0;
    static float pointing_p_gui = 1e-5f;
    static float pointing_i_gui = 1e-7f;

    // window management
    static bool show_demo_window = false;
    static bool show_app_metrics = false;

    // Shear
    if (shear_loop_select == 2) {
      res.metrology.shear_x1_loop.control_mode.store(4);
      res.metrology.shear_x2_loop.control_mode.store(4);
      res.metrology.shear_y1_loop.control_mode.store(4);
      res.metrology.shear_y2_loop.control_mode.store(4);
    } else if (shear_loop_select == 1) {
      res.metrology.shear_x1_loop.control_mode.store(1);
      res.metrology.shear_x2_loop.control_mode.store(1);
      res.metrology.shear_y1_loop.control_mode.store(1);
      res.metrology.shear_y2_loop.control_mode.store(1);
    } else {
      res.metrology.shear_x1_loop.control_mode.store(0);
      res.metrology.shear_x2_loop.control_mode.store(0);
      res.metrology.shear_y1_loop.control_mode.store(0);
      res.metrology.shear_y2_loop.control_mode.store(0);
    }

    res.metrology.shear_x1_loop.p.store(shear_p_gui);
    res.metrology.shear_x1_loop.i.store(shear_i_gui);
    res.metrology.shear_x2_loop.p.store(shear_p_gui);
    res.metrology.shear_x2_loop.i.store(shear_i_gui);
    res.metrology.shear_y1_loop.p.store(shear_p_gui);
    res.metrology.shear_y1_loop.i.store(shear_i_gui);
    res.metrology.shear_y2_loop.p.store(shear_p_gui);
    res.metrology.shear_y2_loop.i.store(shear_i_gui);

    res.metrology.shear_x1_loop.setpoint.store(shear_x1_setpoint_gui);
    res.metrology.shear_x2_loop.setpoint.store(shear_x2_setpoint_gui);
    res.metrology.shear_y1_loop.setpoint.store(shear_y1_setpoint_gui);
    res.metrology.shear_y2_loop.setpoint.store(shear_y2_setpoint_gui);

    // Pointing
    if (pointing_loop_select == 2) {
      res.metrology.point_1_loop.plant_mode.store({2, 2});
      res.metrology.point_1_loop.controller_mode.store({1, 1});
      res.metrology.point_2_loop.plant_mode.store({2, 2});
      res.metrology.point_2_loop.controller_mode.store({1, 1});
    } else if (pointing_loop_select == 1) {
      res.metrology.point_1_loop.plant_mode.store({1, 1});
      res.metrology.point_1_loop.controller_mode.store({0, 0});
      res.metrology.point_2_loop.plant_mode.store({1, 1});
      res.metrology.point_2_loop.controller_mode.store({0, 0});
    } else {
      res.metrology.point_1_loop.plant_mode.store({0, 0});
      res.metrology.point_1_loop.controller_mode.store({0, 0});
      res.metrology.point_2_loop.plant_mode.store({0, 0});
      res.metrology.point_2_loop.controller_mode.store({0, 0});
    }

    res.metrology.point_1_loop.Ps.store({pointing_p_gui, pointing_p_gui});
    res.metrology.point_1_loop.Is.store({pointing_i_gui, pointing_i_gui});
    res.metrology.point_2_loop.Ps.store({pointing_p_gui, pointing_p_gui});
    res.metrology.point_2_loop.Is.store({pointing_i_gui, pointing_i_gui});

    res.metrology.point_1_loop.setpoint.store({{pointing_x1_setpoint_gui, pointing_y1_setpoint_gui}});
    res.metrology.point_2_loop.setpoint.store({{pointing_x2_setpoint_gui, pointing_y2_setpoint_gui}});

    static ScrollingBuffer opd_buffer, shear_x1_buffer, shear_x2_buffer, shear_y1_buffer, shear_y2_buffer,
        point_x1_buffer, point_x2_buffer, point_y1_buffer, point_y2_buffer, sci_null_buffer;

    // GUI interface to save OPD measurements to file:
    // a button "start recording", which when pressed starts recording OPD measurements to a file
    // the same button reads "stop recording" when recording is active
    // when "stop recording" is pressed, the file is closed and the button reads "start recording" again
    // filename is "opd_iso_date_time.csv"
    // data format: time, OPD
    static bool recording_running = false;
    static std::ofstream file;
    static std::string filename;
    if (recording_running) {
      if (ImGui::Button("OPD: Stop recording")) {
        recording_running = false;
        file.close();
      }
    } else {
      if (ImGui::Button("OPD: Start recording")) {
        recording_running = true;
        filename = "measurements/" + utils::get_iso_datestring() + "_opd.csv";
        file.open(filename);
        file << "Time at start of measurement: " << utils::get_iso_datestring() << "\n";
        file << "Time (s),OPD (nm)\n";
      }
    }

    // save intensity measurements to file
    static bool record_sci_null = false;
    static std::ofstream sci_null_file;
    static std::string sci_null_filename;
    if (record_sci_null) {
      if (ImGui::Button("Intensity: Stop recording")) {
        record_sci_null = false;
        sci_null_file.close();
      }
    } else {
      if (ImGui::Button("Intensity: Start recording")) {
        record_sci_null = true;
        sci_null_filename = "measurements/" + utils::get_iso_datestring() + "_sci_null.csv";
        sci_null_file.open(sci_null_filename);
        sci_null_file << "Time at start of measurement: " << utils::get_iso_datestring() << "\n";
        sci_null_file << "Time (s),Intensity\n";
      }
    }

    // add the enture sensor data queue to the plot buffers
    if (!res.metrology.sensorDataQueue.isempty()) {
      int N = res.metrology.sensorDataQueue.size();
      for (int i = 0; i < N; i++) {
        auto m = res.metrology.sensorDataQueue.pop();
        opd_buffer.AddPoint(m.time, m.opd);
        shear_x1_buffer.AddPoint(m.time, m.shear_x1);
        shear_x2_buffer.AddPoint(m.time, m.shear_x2);
        shear_y1_buffer.AddPoint(m.time, m.shear_y1);
        shear_y2_buffer.AddPoint(m.time, m.shear_y2);
        point_x1_buffer.AddPoint(m.time, m.point_x1);
        point_x2_buffer.AddPoint(m.time, m.point_x2);
        point_y1_buffer.AddPoint(m.time, m.point_y1);
        point_y2_buffer.AddPoint(m.time, m.point_y2);
        sci_null_buffer.AddPoint(m.time, m.sci_null);

        // If saving data, write to file
        if (recording_running) {
          file << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(2) << m.opd
               << "\n";
        }

        if (record_sci_null) {
          sci_null_file << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(1)
                        << m.sci_null << "\n";
        }
      }
    }

    // Button to start/stop metrology_reader
    static bool metrology_running = true;
    if (metrology_running) {
      if (ImGui::Button("Metrology: Stop")) {
        metrology_running = false;
        workers.metrology_reader.stop();
      }
    } else {
      if (ImGui::Button("Metrology: Start")) {
        metrology_running = true;
        workers.metrology_reader.start();
      }
    }

    static float t_gui = 0;
    t_gui = utils::getTime();

    // science camera
    if (ImGui::CollapsingHeader("Science Camera")) {
      static CameraInterface cam;
      static std::vector<std::string> command_list = cam.get_commands();

      // buttons for all found commands
      ImGui::Text("Auto-found commands:");
      ImGui::SameLine();
      for (const auto &command : command_list) {
        if (ImGui::Button(command.c_str())) {
          cam.run_command(command);
        }
        ImGui::SameLine();
      }
      ImGui::NewLine();

      static int width, height;
      width = cam.get_width();
      height = cam.get_height();
      ImGui::Text("Image size: %d x %d", width, height);

      static std::vector<unsigned short> image;
      image = cam.get_image();

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

      // get pointer to first element of image
      unsigned short *values = image.data();

      // calculate height and width of plot window
      float aspect_ratio = float(width) / float(height);
      float plot_width = 1000;
      float plot_height = plot_width / aspect_ratio;

      // selection rectangle
      static ImPlotRect rect = {0, double(width) / 2, 0, double(height) / 2};  // {X.Min, X.Max, Y.Min, Y.Max}
      static bool rect_clicked = false, rect_hovered = false, rect_held = false;
      ImPlotDragToolFlags flags = ImPlotDragToolFlags_None;

      // colormap settings
      static int scale_min = 0;
      static int scale_max = 16383;
      const int scale_max_default = 16383;
      const int scale_min_default = 0;
      ImGui::SetNextItemWidth(400);
      ImGui::SliderScalar("Min", ImGuiDataType_U16, &scale_min, &scale_min_default, &scale_max_default, "%u");
      ImGui::SetNextItemWidth(400);
      ImGui::SliderScalar("Max", ImGuiDataType_U16, &scale_max, &scale_min_default, &scale_max_default, "%u");

      // clamp colormap such that min < max
      if (scale_min > scale_max) {
        scale_min = scale_max;
      } else if (scale_max < scale_min) {
        scale_max = scale_min;
      } else if (scale_min == scale_max) {
        scale_min = scale_max - 1;
      }

      // plot image
      if (ImPlot::BeginPlot("##Heatmap2", ImVec2(plot_width, plot_height), ImPlotFlags_NoMouseText)) {
        ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_NoDecorations, ImPlotAxisFlags_NoDecorations);
        ImPlot::PlotHeatmap("heat1", values, height, width, scale_min, scale_max, nullptr, ImPlotPoint(0, 0),
                            ImPlotPoint(width, height));
        ImPlot::DragRect(0, &rect.X.Min, &rect.Y.Min, &rect.X.Max, &rect.Y.Max, ImVec4(1, 0, 0, 0.5), flags,
                         &rect_clicked, &rect_hovered, &rect_held);
        ImPlot::EndPlot();
      }
      ImPlot::PopColormap();

      // get rect coordinates as integers and flip y axis
      int x_min = static_cast<int>(rect.X.Min);
      int x_max = static_cast<int>(rect.X.Max);
      int y_min = static_cast<int>(rect.Y.Min);
      int y_max = static_cast<int>(rect.Y.Max);
      int x_extent = x_max - x_min;
      int y_extent = y_max - y_min;
      y_min += y_extent;
      y_max -= y_extent;
      y_min = height - y_min;
      y_max = height - y_max;

      // Calculate mean intensity in the selected region
      static ScrollingBufferT<double, double> mean_intensity_buffer(1000);
      if (width > 0 && height > 0 && !image.empty()) {
        // Ensure bounds
        x_min = std::max(0, std::min(x_min, width - 1));
        x_max = std::max(0, std::min(x_max, width - 1));
        y_min = std::max(0, std::min(y_min, height - 1));
        y_max = std::max(0, std::min(y_max, height - 1));

        double sum = 0;
        int count = 0;

        for (int y = y_min; y <= y_max; y++) {
          for (int x = x_min; x <= x_max; x++) {
            sum += values[x + y * width];  // row major order
            // example: to get value at (5,0), get values[5]
            // example: to get value at (0,1), get values[width]
            count++;
          }
        }

        if (count > 0) {
          double mean = sum / count;
          ImGui::Text("Mean intensity: %.2f", mean);
          mean_intensity_buffer.AddPoint(t_gui, mean);
        }
      }

      ImGui::Text("Selection: (%d,%d) to (%d,%d)  Size: %d x %d pixels", x_min, y_min, x_max, y_max, x_extent,
                  y_extent);

      // plot a time series of the mean intensity
      static float mean_intensity_history_length = 10.f;
      ImGui::SliderFloat("Mean Intensity History", &mean_intensity_history_length, 1, 100, "%.5f s",
                         ImGuiSliderFlags_Logarithmic);
      if (ImPlot::BeginPlot("Mean Intensity", ImVec2(-1, 400 * io.FontGlobalScale))) {
        ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxisLimits(ImAxis_X1, t_gui - mean_intensity_history_length, t_gui, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
        ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, 2);
        // ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
        ImPlot::PlotLine("Mean Intensity", &mean_intensity_buffer.Data[0].time, &mean_intensity_buffer.Data[0].value,
                         mean_intensity_buffer.Data.size(), 0, mean_intensity_buffer.Offset, 2 * sizeof(double));
        ImPlot::EndPlot();
      }
    }

    // ADC measurements
    if (ImGui::CollapsingHeader("ADC Measurements")) {
      static ScrollingBufferT<int, int> adc_buffers[14];
      static float t_adc = 0;

      // get lates time in ADC queue

      if (!res.metrology.adc_queues[0].isempty()) {
        t_adc = res.metrology.adc_queues[0].back().time;
      }

      // add all measurements to the plot buffers
      for (int i = 0; i < 14; i++) {
        if (!res.metrology.adc_queues[i].isempty()) {
          int N = res.metrology.adc_queues[i].size();
          for (int j = 0; j < N; j++) {
            auto m = res.metrology.adc_queues[i].pop();
            adc_buffers[i].AddPoint(m.time, m.value);
          }
        }
      }

      static float adc_history_length = 1000.f;
      ImGui::SliderFloat("ADC History", &adc_history_length, 1, 128000, "%.5f s", ImGuiSliderFlags_Logarithmic);

      // plot label names
      static const char *plot_labels[14] = {"Shear UP",   "Shear LEFT",  "Shear RIGHT", "Shear DOWN", "Point UP",
                                            "Point LEFT", "Point RIGHT", "Point DOWN",  "SineRef",    "OPDRef",
                                            "Shear Sum",  "Point Sum",   "SCINull",     "SCIMod"};

      // plot style
      static float thickness = 2;
      static ImPlotAxisFlags xflags = ImPlotAxisFlags_NoTickLabels;
      static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;

      if (ImPlot::BeginPlot("##ADC", ImVec2(-1, 400 * io.FontGlobalScale))) {
        ImPlot::SetupAxes(nullptr, nullptr, xflags, yflags);
        ImPlot::SetupAxisLimits(ImAxis_X1, t_adc - adc_history_length, t_adc, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
        for (int i = 0; i < 14; i++) {
          ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(i), thickness);
          ImPlot::PlotLine(plot_labels[i], &adc_buffers[i].Data[0].time, &adc_buffers[i].Data[0].value,
                           adc_buffers[i].Data.size(), 0, adc_buffers[i].Offset, 2 * sizeof(int));
        }
        ImPlot::EndPlot();
      }
    }

    // Sci_null measurements
    if (ImGui::CollapsingHeader("Science beam Measurements")) {
      // history length slider
      static float sci_null_history_length = 5.0;
      ImGui::SliderFloat("Sci_null History", &sci_null_history_length, 1e-3, 50., "%.3f s",
                         ImGuiSliderFlags_Logarithmic);

      // plot time series
      static float thickness = 3;
      static ImPlotAxisFlags xflags = ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoTickLabels;
      static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;

      // print current measurement
      if (!sci_null_buffer.Data.empty()) {
        auto m = sci_null_buffer.Data.back();
        ImGui::Text("Current Sci_null: %.1f", m.y);
      }

      if (ImPlot::BeginPlot("##Sci_null", ImVec2(-1, 600 * io.FontGlobalScale))) {
        ImPlot::SetupAxes(nullptr, nullptr, xflags, yflags);
        ImPlot::SetupAxisLimits(ImAxis_X1, t_gui - sci_null_history_length, t_gui, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(0), thickness);
        ImPlot::PlotLine("Sci_null", &sci_null_buffer.Data[0].x, &sci_null_buffer.Data[0].y,
                         sci_null_buffer.Data.size(), 0, sci_null_buffer.Offset, 2 * sizeof(int));
        ImPlot::EndPlot();
      }

      // calculate fft
      const static int fft_size = 1024 * 8 * 8;
      static double fft_power[fft_size / 2];
      static double fft_freq[fft_size / 2];
      static FFT_calculator fft(fft_size, 12800., &sci_null_buffer, fft_power, fft_freq);
      fft.calculate();

      // plot fft
      static ImVec4 fft_color = ImVec4(1, 1, 0, 1);
      static float fft_thickness = 3;
      static ImPlotAxisFlags fft_xflags = ImPlotAxisFlags_None;
      static ImPlotAxisFlags fft_yflags = ImPlotAxisFlags_None;  // ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
      if (ImPlot::BeginPlot("##FFT", ImVec2(-1, 400 * io.FontGlobalScale))) {
        ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
        ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
        ImPlot::SetupAxes(nullptr, nullptr, fft_xflags, fft_yflags);
        ImPlot::SetupAxisLimits(ImAxis_X1, 0.1, 2000);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 10, 1e13);
        ImPlot::SetNextLineStyle(fft_color, fft_thickness);
        ImPlot::PlotLine("Sci Null FFT", &fft_freq[0], &fft_power[0], fft_size / 2, 0, 0, 8);
        ImPlot::EndPlot();
      }
    }

    if (ImGui::CollapsingHeader("OPD")) {
      if (ImGui::TreeNode("Measurement##OPD")) {
        // phase unwrapping
        static bool reset_phase_unwrap_gui = false;
        ImGui::Checkbox("Reset phase unwrap", &reset_phase_unwrap_gui);
        if (reset_phase_unwrap_gui) {
          res.metrology.reset_phase_unwrap.store(1);
        } else {
          res.metrology.reset_phase_unwrap.store(0);
        }

        static float opd_history_length = 10.0f;
        ImGui::SliderFloat("OPD History", &opd_history_length, 0.1, 5, "%.2f s", ImGuiSliderFlags_Logarithmic);

        // x axis: no ticks
        static ImPlotAxisFlags xflags = ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoTickLabels;

        // y axis: auto fit
        static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;

        static float thickness = 1;

        if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1, 200 * io.FontGlobalScale))) {
          ImPlot::SetupAxes(nullptr, nullptr, xflags, yflags);
          ImPlot::SetupAxisLimits(ImAxis_X1, t_gui - opd_history_length, t_gui, ImGuiCond_Always);
          ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
          ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
          ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(1), thickness);
          ImPlot::PlotLine("Measurement", &opd_buffer.Data[0].x, &opd_buffer.Data[0].y, opd_buffer.Data.size(), 0,
                           opd_buffer.Offset, 2 * sizeof(float));
          ImPlot::EndPlot();
        }

        // calculate mean and std of OPD buffer
        static float mean = 0.0f;
        static float stddev = 0.0f;
        const int N_points_stats = 1000;

        if (opd_buffer.Data.size() > N_points_stats + 2) {
          // get the last 1000 measurements using ImVector<ImVec2> GetLastN(int n)
          auto last_1000_measurements_data = opd_buffer.GetLastN(N_points_stats);
          // get the y values
          std::vector<float> last_1000_measurements;
          for (auto &m : last_1000_measurements_data) {
            last_1000_measurements.push_back(m.y);
          }

          // calculate mean and stdev
          mean = std::accumulate(last_1000_measurements.begin(), last_1000_measurements.end(), 0.0) /
                 last_1000_measurements.size();
          float sq_sum = std::inner_product(last_1000_measurements.begin(), last_1000_measurements.end(),
                                            last_1000_measurements.begin(), 0.0);
          stddev = std::sqrt(sq_sum / last_1000_measurements.size() - mean * mean);
        }

        // Display mean and std
        ImGui::Text("Mean: %.4f", mean);
        ImGui::SameLine();
        ImGui::Text("Std: %.4f", stddev);

        ImGui::TreePop();
      }

      if (ImGui::TreeNode("FFT##OPD")) {
        // set up fft
        const static int fft_size = 1024 * 8 * 8;
        static double fft_power[fft_size / 2];
        static double fft_freq[fft_size / 2];
        static FFT_calculator fft(fft_size, 12800., &opd_buffer, fft_power, fft_freq);

        // calculate fft
        fft.calculate();

        // plot fft_power vs fft_freq, with log scale on x and y axis
        static float fft_thickness = 3;
        static ImPlotAxisFlags fft_xflags = ImPlotAxisFlags_None;
        static ImPlotAxisFlags fft_yflags =
            ImPlotAxisFlags_None;  // ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
        if (ImPlot::BeginPlot("##FFT", ImVec2(-1, 400 * io.FontGlobalScale))) {
          ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
          ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
          ImPlot::SetupAxes(nullptr, nullptr, fft_xflags, fft_yflags);
          ImPlot::SetupAxisLimits(ImAxis_X1, 0.1, 2000);
          ImPlot::SetupAxisLimits(ImAxis_Y1, 10, 1e13);
          ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(1), fft_thickness);
          ImPlot::PlotLine("OPD FFT", &fft_freq[0], &fft_power[0], fft_size / 2);
          ImPlot::EndPlot();
        }

        ImGui::TreePop();
      }

      if (ImGui::TreeNode("Control##OPD")) {
        static int gui_opd_loop_select = 0;
        ImGui::Text("Control mode:");
        ImGui::SameLine();
        ImGui::RadioButton("Off##OPD", &gui_opd_loop_select, 0);
        ImGui::SameLine();
        // ImGui::RadioButton("Open loop##OPD", &gui_opd_loop_select, 1);
        // ImGui::SameLine();
        ImGui::RadioButton("Closed loop##OPD", &gui_opd_loop_select, 2);

        // OPD setpoint
        static float opd_setpoint_ethercat = 0.0f;
        const float opd_setpoint_min = -1e6, opd_setpoint_max = 1e6;
        ImGui::DragFloat("OPD Setpoint", &opd_setpoint_ethercat, 0.1f, opd_setpoint_min, opd_setpoint_max, "%.2f nm",
                         ImGuiSliderFlags_AlwaysClamp);

        // P and I control loop gains
        static float opd_p_ethercat = 0.0f;
        static float opd_i_ethercat = 100.0f;
        ImGui::SliderFloat("P##OPD EtherCAT", &opd_p_ethercat, 1e-4f, 1e0f, "%.5f", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderFloat("I##OPD EtherCAT", &opd_i_ethercat, 1e-1f, 1e3f, "%.5f", ImGuiSliderFlags_Logarithmic);

        // two binary flags: Reset phase unwrap, and run control loop
        static bool reset_phase_unwrap = false;
        ImGui::Checkbox("Reset phase unwrap", &reset_phase_unwrap);

        // Interface to EtherCAT master: Send a few bytes via UDP to the master receiver
        // 5 bytes data via UDP:
        // 4 bytes: OPD setpoint float
        // 1 byte: flags: X X X X X X reset_phase_unwrap run_control_loop
        const int udp_port = 8888;
        const char *udp_ip = "192.168.88.177";
        static EthercatUdpInterface ec_udp_if(udp_ip, udp_port);
        ec_udp_if.send_commands(opd_setpoint_ethercat * 1.0e-3, opd_p_ethercat, opd_i_ethercat, reset_phase_unwrap,
                                gui_opd_loop_select == 2);

        ImGui::TreePop();
      }
    }

    if (ImGui::CollapsingHeader("Shear")) {
      static float t_gui_x = 0;

      // if measurement is running, update gui time.
      t_gui_x = utils::getTime();

      static float x1_history_length = 10.0f;
      ImGui::SliderFloat("History", &x1_history_length, 0.1, 10, "%.2f s", ImGuiSliderFlags_Logarithmic);

      // x axis: no ticks
      static ImPlotAxisFlags x1_xflags = ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoTickLabels;

      // y axis: auto fit
      static ImPlotAxisFlags x1_yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;

      float x1_thickness = 3 * io.FontGlobalScale;

      // plot shear
      if (ImPlot::BeginPlot("##X1", ImVec2(-1, 200 * io.FontGlobalScale))) {
        ImPlot::SetupAxes(nullptr, nullptr, x1_xflags, x1_yflags);
        ImPlot::SetupAxisLimits(ImAxis_X1, t_gui_x - x1_history_length, t_gui_x, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(0), x1_thickness);
        ImPlot::PlotLine("Shear X1", &shear_x1_buffer.Data[0].x, &shear_x1_buffer.Data[0].y,
                         shear_x1_buffer.Data.size(), 0, shear_x1_buffer.Offset, 2 * sizeof(float));
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(1), x1_thickness);
        ImPlot::PlotLine("Shear Y1", &shear_y1_buffer.Data[0].x, &shear_y1_buffer.Data[0].y,
                         shear_y1_buffer.Data.size(), 0, shear_y1_buffer.Offset, 2 * sizeof(float));
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(2), x1_thickness);
        ImPlot::PlotLine("Shear X2", &shear_x2_buffer.Data[0].x, &shear_x2_buffer.Data[0].y,
                         shear_x2_buffer.Data.size(), 0, shear_x2_buffer.Offset, 2 * sizeof(float));
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(3), x1_thickness);
        ImPlot::PlotLine("Shear Y2", &shear_y2_buffer.Data[0].x, &shear_y2_buffer.Data[0].y,
                         shear_y2_buffer.Data.size(), 0, shear_y2_buffer.Offset, 2 * sizeof(float));
        ImPlot::EndPlot();
      }

      // calculate std and rms of x1d over last 1000 points
      static float x1d_std = 0.0f;
      static float x1d_rms = 0.0f;
      // static float x2d_std = 0.0f;
      // static float x2d_rms = 0.0f;

      if (shear_x1_buffer.Data.size() > 1000) {
        // calculate mean
        float sum = 0.0f;
        for (auto &p : shear_x1_buffer.Data) {
          sum += p.y;
        }
        float mean = sum / shear_x1_buffer.Data.size();

        // calculate std
        float sum_sq = 0.0f;
        for (auto &p : shear_x1_buffer.Data) {
          sum_sq += (p.y - mean) * (p.y - mean);
        }
        x1d_std = sqrt(sum_sq / shear_x1_buffer.Data.size());

        // calculate rms
        sum_sq = 0.0f;
        for (auto &p : shear_x1_buffer.Data) {
          sum_sq += p.y * p.y;
        }
        x1d_rms = sqrt(sum_sq / shear_x1_buffer.Data.size());
      }

      // print it
      ImGui::Text("x1d std: %.4f", x1d_std);
      ImGui::Text("x1d rms: %.4f", x1d_rms);

      // FFT
      if (ImGui::TreeNode("FFT##Shear")) {
        // set up fft of x1d
        const static int fft_size = 1024 * 8 * 8;
        static double fft_power[fft_size / 2];
        static double fft_freq[fft_size / 2];
        static FFT_calculator fft(fft_size, 12800., &shear_x1_buffer, fft_power, fft_freq);

        // calculate fft
        fft.calculate();

        static float fft_thickness = 3;
        ImVec4 fft_color = ImVec4(1, 1, 0, 1);

        // x axis: no flags
        static ImPlotAxisFlags fft_xflags = ImPlotAxisFlags_None;
        static ImPlotAxisFlags fft_yflags =
            ImPlotAxisFlags_None;  // ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
        // plot fft_power vs fft_freq, with log scale on x and y axis
        if (ImPlot::BeginPlot("##FFT_x1d", ImVec2(-1, 300 * io.FontGlobalScale))) {
          ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
          // ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
          // yflags
          ImPlot::SetupAxes(nullptr, nullptr, fft_xflags, fft_yflags);
          ImPlot::SetupAxisLimits(ImAxis_X1, 10, 3200);
          ImPlot::SetupAxisLimits(ImAxis_Y1, 0.1, 1e8);
          // ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
          ImPlot::SetNextLineStyle(fft_color, fft_thickness);
          ImPlot::PlotLine("FFT", &fft_freq[0], &fft_power[0], fft_size / 2);
          ImPlot::EndPlot();
        }
        ImGui::TreePop();
      }

    }  // end of x position

    if (ImGui::CollapsingHeader("Pointing")) {
      // control mode selector
      ImGui::Text("Control mode:");
      ImGui::SameLine();
      ImGui::RadioButton("Off##Pointing", &pointing_loop_select, 0);
      ImGui::SameLine();
      ImGui::RadioButton("Open loop##Pointing", &pointing_loop_select, 1);
      ImGui::SameLine();
      ImGui::RadioButton("Closed loop##Pointing", &pointing_loop_select, 2);

      static float t_gui_x = 0;

      // if measurement is running, update gui time.
      t_gui_x = utils::getTime();

      static float pointing_history_length = 10.0f;
      ImGui::SliderFloat("History", &pointing_history_length, 0.1, 10, "%.2f s", ImGuiSliderFlags_Logarithmic);

      static ImPlotAxisFlags x1_xflags = ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoTickLabels;
      static ImPlotAxisFlags x1_yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;

      float pointing_thickness = 3 * io.FontGlobalScale;

      // plot
      if (ImPlot::BeginPlot("##Pointing", ImVec2(-1, 200 * io.FontGlobalScale))) {
        ImPlot::SetupAxes(nullptr, nullptr, x1_xflags, x1_yflags);
        ImPlot::SetupAxisLimits(ImAxis_X1, t_gui_x - pointing_history_length, t_gui_x, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(0), pointing_thickness);
        ImPlot::PlotLine("Pointing X1", &point_x1_buffer.Data[0].x, &point_x1_buffer.Data[0].y,
                         point_x1_buffer.Data.size(), 0, point_x1_buffer.Offset, 2 * sizeof(float));
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(1), pointing_thickness);
        ImPlot::PlotLine("Pointing Y1", &point_y1_buffer.Data[0].x, &point_y1_buffer.Data[0].y,
                         point_y1_buffer.Data.size(), 0, point_y1_buffer.Offset, 2 * sizeof(float));
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(2), pointing_thickness);
        ImPlot::PlotLine("Pointing X2", &point_x2_buffer.Data[0].x, &point_x2_buffer.Data[0].y,
                         point_x2_buffer.Data.size(), 0, point_x2_buffer.Offset, 2 * sizeof(float));
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(3), pointing_thickness);
        ImPlot::PlotLine("Pointing Y2", &point_y2_buffer.Data[0].x, &point_y2_buffer.Data[0].y,
                         point_y2_buffer.Data.size(), 0, point_y2_buffer.Offset, 2 * sizeof(float));
        ImPlot::EndPlot();
      }
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
