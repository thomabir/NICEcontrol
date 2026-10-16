#pragma once

#include <GLFW/glfw3.h>

#include <cstdio>
#include <iostream>
#include <thread>

#include "core/Core.hpp"
#include "gui/AdcPanel.hpp"
#include "gui/CameraPanel.hpp"
#include "gui/CorePanel.hpp"
#include "gui/EthercatPanel.hpp"
#include "gui/OpdPanel.hpp"
#include "gui/PlotHistory.hpp"
#include "gui/TangoPanel.hpp"
#include "gui/TipTiltPanel.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "lib/fonts/SourceSans3Regular.cpp"  // NOLINT(bugprone-suspicious-include): the font data, made by binary_to_compressed_c
#include "lib/implot/implot.h"

// The user interface. It shows what the core publishes and it sends commands to the core. It never touches hardware,
// so the program keeps running with no user interface open. This class owns the window and the frame loop, and each
// panel draws one subsystem.
class NiceGui {
 public:
  explicit NiceGui(Core &control_core)
      : core(control_core),
        history(control_core.whiteboard()),
        tango(control_core.commands()),
        camera(control_core.commands(), control_core.whiteboard(), history),
        adc(history),
        ethercat(history),
        opd(control_core.commands(), history),
        tiptilt(control_core.commands()) {}

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
  Core &core;

  Snapshot snap;
  std::jthread gui_thread;
  GLFWwindow *window = nullptr;
  ImGuiIO *io = nullptr;
  ImFont *mainFont = nullptr;
  ImVec4 clear_color;

  PlotHistory history;
  CorePanel core_panel;
  TangoPanel tango;
  CameraPanel camera;
  AdcPanel adc;
  EthercatPanel ethercat;
  OpdPanel opd;
  TipTiltPanel tiptilt;

  void RenderFrame() {
    snap = core.whiteboard().snapshot();
    history.Drain(snap);

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

    tango.RenderRestartBar();
    if (ImGui::CollapsingHeader("Core", ImGuiTreeNodeFlags_DefaultOpen)) {
      core_panel.Render(snap);
    }
    if (ImGui::CollapsingHeader("Tango devices", ImGuiTreeNodeFlags_DefaultOpen)) {
      tango.RenderDevices(snap);
    }
    if (ImGui::CollapsingHeader("Flir Camera")) {
      camera.Render(snap);
    }
    if (ImGui::CollapsingHeader("ADC Measurements")) {
      adc.Render();
    }
    if (ImGui::CollapsingHeader("EtherCAT Monitor")) {
      ethercat.Render();
    }
    if (ImGui::CollapsingHeader("OPD")) {
      opd.Render(snap);
    }
    if (ImGui::CollapsingHeader("Lateral beam control")) {
      tiptilt.Render(snap);
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

  bool Initialize() {
    glfwSetErrorCallback(glfw_error_callback);

    // GLFW 3.4 probes Wayland before X11, printing "XDG_RUNTIME_DIR not set"
    // on systems where Wayland is not running. Force X11 directly.
    glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);

    if (!glfwInit()) return false;

    // OpenGL 3.0 with GLSL 1.30.
    constexpr const char *kGlslVersion = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

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
    ImGui_ImplOpenGL3_Init(kGlslVersion);

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
