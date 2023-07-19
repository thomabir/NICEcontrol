#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h>  // Will drag system OpenGL headers
#include <math.h>
#include <stdio.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <mutex>
#include <queue>
#include <random>
#include <thread>

// ethernet
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

// MCL piezo stage
#include "../lib/madlib/madlib.h"

#include <cstring>

#include "../lib/fonts/SourceSans3Regular.cpp"
#include "../lib/implot/implot.h"

// Windows
#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

static void glfw_error_callback(int error, const char *description) {
  fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

float getTime() {
  // returns time in seconds since the start of the program
  static auto t0 = std::chrono::system_clock::now();
  auto tnow = std::chrono::system_clock::now();
  float t_since_start = std::chrono::duration_cast<std::chrono::microseconds>(tnow - t0).count() / 1.0e6;
  return t_since_start;
}

struct Measurement {
  float time;
  float value;
};

// thread-safe queue
template <typename T>
class TSQueue {
 private:
  std::queue<T> m_queue;
  std::mutex m_mutex;

 public:
  uint size() {
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_queue.size();
  }

  void push(T item) {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_queue.push(item);
  }

  T front() {
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_queue.front();
  }

  T back() {
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_queue.back();
  }

  T pop() {
    std::unique_lock<std::mutex> lock(m_mutex);
    T item = m_queue.front();
    m_queue.pop();
    return item;
  }

  bool isempty() {
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_queue.empty();
  }
};

struct ScrollingBuffer {
  int MaxSize;
  int Offset;
  ImVector<ImVec2> Data;
  ScrollingBuffer(int max_size = 10000) {
    MaxSize = max_size;
    Offset = 0;
    Data.reserve(MaxSize);
  }
  void AddPoint(float x, float y) {
    if (Data.size() < MaxSize)
      Data.push_back(ImVec2(x, y));
    else {
      Data[Offset] = ImVec2(x, y);
      Offset = (Offset + 1) % MaxSize;
    }
  }
  void Erase() {
    if (Data.size() > 0) {
      Data.shrink(0);
      Offset = 0;
    }
  }
};

namespace MyApp {

// OPD control
float opd_setpoint_gui = 0.001f;  // setpoint entered in GUI, may be out of range
std::atomic<float> opd_setpoint = 0.001f;      // setpoint used in calculation, clipped to valid range
std::atomic<bool> RunOpdControl(false);
std::mutex OpdControlMutex;
std::condition_variable OpdControlCV;
std::atomic<float> p = 0.0f;
std::atomic<float> i = 0.0f;

// OPD measurement
std::atomic<bool> RunOpdMeasurement(false);
std::mutex OpdMeasurementMutex;
std::condition_variable OpdMeasurementCV;
std::ofstream outputFile;

// OPD actuator
int initialise_opd_stage(){
  int handle = MCL_InitHandle();
  // move to a safe central position
  MCL_SingleWriteN(25., 3, handle);
  return handle;
}
int handle = initialise_opd_stage();
std::atomic<float> piezo_setpoint = 0.0f;

TSQueue<Measurement> measurementQueue;

// void run_calculation() {
//   static float measurement = 0.0f;

//   // outputFile.open("data.csv");
//   // outputFile << "Time (ms),Measurement\n";

//   while (true) {
//     {
//       std::unique_lock<std::mutex> lock(OpdMeasurementMutex);
//       OpdMeasurementCV.wait(lock, [] { return RunOpdMeasurement.load(); });
//     }

//     // generate mock measurement with gaussian noise
//     static double mean = 0.0;
//     static double stddev = 0.1;
//     static std::default_random_engine generator;
//     static std::normal_distribution<double> dist(mean, stddev);
//     auto noise = dist(generator);
//     measurement = opd_setpoint + noise;
//     auto t = getTime();

//     // enqueue measurement and time
//     measurementQueue.push({t, measurement});

//     // Write measurement and time to the CSV file
//     // outputFile << currentTime << "," << measurement.load() << "\n";

//     // wait 100 µs
//     std::this_thread::sleep_for(std::chrono::microseconds(100));
//   }
// }



void opd_move_to(float setpoint) {
  MCL_SingleWriteN(setpoint, 3, handle);
}

void run_calculation() {
  // setup ethernet connection
  // Create a UDP socket
  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    std::cerr << "Failed to create socket." << std::endl;
  }

  // Set up the server address
  struct sockaddr_in serverAddr;
  std::memset(&serverAddr, 0, sizeof(serverAddr));
  serverAddr.sin_family = AF_INET;
  int PortNumber = 12345;
  serverAddr.sin_port = htons(PortNumber);         // Replace with the desired port number
  serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);  // Listen on all network interfaces

  // Bind the socket to the server address
  if (bind(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
    std::cerr << "Failed to bind socket." << std::endl;
  }

  // Open the output file
  std::ofstream outputFile("output.csv");
  if (!outputFile) {
    std::cerr << "Failed to open output file." << std::endl;
  }

  int count = 0;
  int prev = 0;
  int buffer_size = 1024;
  char buffer[buffer_size];

  static float measurement = 0.0f;

  // outputFile.open("data.csv");
  // outputFile << "Time (ms),Measurement\n";

  while (true) {
    {
      std::unique_lock<std::mutex> lock(
          OpdMeasurementMutex);  // the only purpose of that lock is to run/stop measurement?
      OpdMeasurementCV.wait(lock, [] { return RunOpdMeasurement.load(); });
    }

    // read the measurement from the ethernet connection
    count++;

    // Receive data
    struct sockaddr_in clientAddr;
    socklen_t clientAddrLen = sizeof(clientAddr);
    int numBytes = recvfrom(sockfd, buffer, buffer_size, 0, (struct sockaddr *)&clientAddr, &clientAddrLen);

    // Check for errors
    if (numBytes < 0) {
      std::cerr << "Error receiving data." << std::endl;
      break;
    }

    // Convert received data to vector of 10 ints
    int receivedData[10];
    std::memcpy(receivedData, buffer, sizeof(int) * 10);

    // Calculate average of received data
    float sum = 0;
    for (int i = 0; i < 10; i++) {
      sum += receivedData[i];
    }
    float avg = sum / 10.;

    // convert from millidegree to rad
    avg *= 1e-3 * M_PI / 180.;

    // from rad to nm, assuming 633 nm  = 2 pi rad
    avg *= 633. / (2. * M_PI);

    auto t = getTime();

    // enqueue measurement and time
    measurementQueue.push({t, avg});

    // variables for control
    static float opd_error = 0.0f;
    static float opd_error_integral = 0.0f;
    static float opd_control_signal = 0.0f;

    // if RunOpdControl is true, calculate the control signal
    if (RunOpdControl.load()) {
      // calculate error
      opd_error = opd_setpoint.load() - avg;

      // calculate integral
      opd_error_integral += i.load() * opd_error;

      // calculate derivative
      // opd_error_derivative = opd_error - opd_error_prev;

      // calculate control signal
      opd_control_signal = p.load() * opd_error +  opd_error_integral;

      // actuate piezo (takes input in µm)
      opd_move_to(opd_control_signal/1000.);
    }


    // Write measurement and time to the CSV file
    // outputFile << currentTime << "," << measurement.load() << "\n";

    // wait 100 µs
    // std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}

// Presenter
void startOpdMeasurement() {
  RunOpdMeasurement.store(true);
  OpdMeasurementCV.notify_one();
}

void stopOpdMeasurement() { RunOpdMeasurement.store(false); }

void RenderUI() {
  ImGui::Begin("NICE Control");

  ImGuiIO &io = ImGui::GetIO();

  static auto current_measurement = 0.f;
  if (!measurementQueue.isempty()) {
    current_measurement = measurementQueue.back().value;
  }

  if (ImGui::CollapsingHeader("OPD")) {
    // control mode selector
    static int loop_select = 0;
    ImGui::Text("Control mode:");
    ImGui::SameLine();
    ImGui::RadioButton("Off", &loop_select, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Open loop", &loop_select, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Closed loop", &loop_select, 2);

    // real time plot
    static ScrollingBuffer opd_buffer, setpoint_buffer;
    float t_gui = getTime();

    // add the entire MeasurementQueue to the buffer
    // if there's nothing in it, just add a NaN
    if (measurementQueue.isempty()) {
      opd_buffer.AddPoint(t_gui, NAN);
    } else {
      while (!measurementQueue.isempty()) {
        auto m = measurementQueue.pop();
        opd_buffer.AddPoint(m.time, m.value);
      }
    }

    setpoint_buffer.AddPoint(t_gui, opd_setpoint.load());

    static bool plot_setpoint = false;
    ImGui::Checkbox("Plot setpoint", &plot_setpoint);

    static float history_length = 10.0f;
    ImGui::SliderFloat("History", &history_length, 0.1, 10, "%.2f s", ImGuiSliderFlags_Logarithmic);

    // x axis: no ticks
    static ImPlotAxisFlags xflags = ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoTickLabels;

    // y axis: auto fit
    static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;

    if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1, 150 * io.FontGlobalScale))) {
      ImPlot::SetupAxes(nullptr, nullptr, xflags, yflags);
      ImPlot::SetupAxisLimits(ImAxis_X1, t_gui - history_length, t_gui, ImGuiCond_Always);
      ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
      ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
      ImPlot::PlotLine("Measurement", &opd_buffer.Data[0].x, &opd_buffer.Data[0].y, opd_buffer.Data.size(), 0,
                       opd_buffer.Offset, 2 * sizeof(float));
      if (plot_setpoint){
      ImPlot::PlotLine("Setpoint", &setpoint_buffer.Data[0].x, &setpoint_buffer.Data[0].y, setpoint_buffer.Data.size(),
                       0, setpoint_buffer.Offset, 2 * sizeof(float));
      }
      ImPlot::EndPlot();
    }

    if (ImGui::TreeNode("OPD metrology")) {
      // Display measurement
      ImGui::Text("Current measurement: %.4f", current_measurement);

      // calculate mean and std of OPD buffer
      static float mean = 0.0f;
      static float stddev = 0.0f;

      if (opd_buffer.Data.size() > 0) {
        // calculate mean
        float sum = 0.0f;
        for (auto &p : opd_buffer.Data) {
          sum += p.y;
        }
        mean = sum / opd_buffer.Data.size();

        // calculate std
        float sum_sq = 0.0f;
        for (auto &p : opd_buffer.Data) {
          sum_sq += (p.y - mean) * (p.y - mean);
        }
        stddev = sqrt(sum_sq / opd_buffer.Data.size());
      }

      // Display mean and std
      ImGui::Text("Mean: %.4f", mean);
      ImGui::Text("Std: %.4f", stddev);




      ImGui::TreePop();
    }

    if (ImGui::TreeNode("Piezo stage")) {
      float piezo_measurement = 0.0f;

      // Display measurement
      ImGui::Text("Current measurement: %.4f", piezo_measurement);

      // open loop setpoint µm
      static float piezo_setpoint = 0.0f;
      ImGui::SliderFloat("Setpoint", &piezo_setpoint, 0.0f, 50.0f);

      ImGui::TreePop();
    }

    if (ImGui::TreeNode("Control loop")) {
      ImGui::Text("Control loop parameters:");

      // sliders for p , i
      static float p_gui = 0.0f;
      static float i_gui = 0.0f;
      
      ImGui::SliderFloat("P", &p_gui, 0.0f, 1.0f);
      ImGui::SliderFloat("I", &i_gui, 0.0f, 5e-2f);

      // store p, i
      p.store(p_gui);
      i.store(i_gui);

      const float opd_setpoint_min = -1000.0f, opd_setpoint_max = 1000.0f;

      // if (ImGui::Button("Start Calculation")) startOpdMeasurement();
      // if (ImGui::Button("Stop Calculation")) stopOpdMeasurement();
      static bool measure_opd = true;
      ImGui::Checkbox("Run measurement", &measure_opd);
      if (measure_opd) {
        startOpdMeasurement();
      } else {
        stopOpdMeasurement();
      }

      static bool control_opd = false;
      ImGui::Checkbox("Run control", &control_opd);
      if (control_opd) {
        RunOpdControl.store(true);
      } else {
        RunOpdControl.store(false);
      }

      // opd input: drag
      ImGui::SliderFloat("(Drag or double-click to adjust)", &opd_setpoint_gui, opd_setpoint_min,
                       opd_setpoint_max, "%.1f nm", ImGuiSliderFlags_AlwaysClamp);

      // opd input: buttons
      if (ImGui::BeginTable("table1", 6, ImGuiTableFlags_SizingFixedFit)) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::Text("Increase:");
        ImGui::TableNextColumn();
        if (ImGui::Button("+0.1 nm")) {
          opd_setpoint_gui += 0.0001;
        }
        ImGui::TableNextColumn();
        if (ImGui::Button("+1 nm")) {
          opd_setpoint_gui += 0.001;
        }
        ImGui::TableNextColumn();
        if (ImGui::Button("+10 nm")) {
          opd_setpoint_gui += 0.01;
        }
        ImGui::TableNextColumn();
        if (ImGui::Button("+100 nm")) {
          opd_setpoint_gui += 0.1;
        }
        ImGui::TableNextColumn();
        if (ImGui::Button("+1 µm")) {
          opd_setpoint_gui += 1.;
        }

        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::Text("Decrease:");
        ImGui::TableNextColumn();
        if (ImGui::Button("-0.1 nm")) {
          opd_setpoint_gui -= 0.0001;
        }
        ImGui::TableNextColumn();
        if (ImGui::Button("-1 nm")) {
          opd_setpoint_gui -= 0.001;
        }
        ImGui::TableNextColumn();
        if (ImGui::Button("-10 nm")) {
          opd_setpoint_gui -= 0.01;
        }
        ImGui::TableNextColumn();
        if (ImGui::Button("-100 nm")) {
          opd_setpoint_gui -= 0.1;
        }
        ImGui::TableNextColumn();
        if (ImGui::Button("-1 µm")) {
          opd_setpoint_gui -= 1.;
        }

        ImGui::EndTable();
      }

      // clamp opd_setpoint_gui to min/max
      if (opd_setpoint_gui < opd_setpoint_min) opd_setpoint_gui = opd_setpoint_min;
      if (opd_setpoint_gui > opd_setpoint_max) opd_setpoint_gui = opd_setpoint_max;

      // set opd_setpoint
      opd_setpoint.store(opd_setpoint_gui);

      ImGui::TreePop();
    }

    // TODO next: add seperate buttons for start/stop of
    // measurements (piezo pos, opd), and seperate logging of all at their own
    // speeds, in seperate threads
  }

  if (ImGui::CollapsingHeader("Program settings")) {
    ImGui::DragFloat("GUI scale", &io.FontGlobalScale, 0.005f, 0.5, 3.0, "%.2f",
                     ImGuiSliderFlags_AlwaysClamp);  // Scale everything
  }
  ImGui::End();

  // demo window
  ImGui::ShowDemoWindow();

  // implot demo window
  ImPlot::ShowDemoWindow();
}

}  // namespace MyApp

// Main code
int main(int, char **) {
  glfwSetErrorCallback(glfw_error_callback);
  if (!glfwInit()) return 1;

    // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
  // GL ES 2.0 + GLSL 100
  const char *glsl_version = "#version 100";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
  // GL 3.2 + GLSL 150
  const char *glsl_version = "#version 150";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
  // GL 3.0 + GLSL 130
  const char *glsl_version = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  // glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+
  // only glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // 3.0+ only
#endif

  // Create window with graphics context
  GLFWwindow *window = glfwCreateWindow(1280, 720, "NICEcontrol", nullptr, nullptr);
  if (window == nullptr) return 1;
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);  // Enable vsync

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;   // Enable Gamepad Controls
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;      // Enable Docking
  io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;    // Enable Multi-Viewport
                                                         // / Platform Windows
  // io.ConfigViewportsNoAutoMerge = true;
  // io.ConfigViewportsNoTaskBarIcon = true;

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();
  // ImGui::StyleColorsLight();

  // When viewports are enabled we tweak WindowRounding/WindowBg so platform
  // windows can look identical to regular ones.
  ImGuiStyle &style = ImGui::GetStyle();
  if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
    style.WindowRounding = 5.0f;
    style.Colors[ImGuiCol_WindowBg].w = 1.0f;
  }

  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  // Load Fonts
  // - If no fonts are loaded, dear imgui will use the default font. You can
  // also load multiple fonts and use ImGui::PushFont()/PopFont() to select
  // them.
  // - AddFontFromFileTTF() will return the ImFont* so you can store it if you
  // need to select the font among multiple.
  // - If the file cannot be loaded, the function will return a nullptr. Please
  // handle those errors in your application (e.g. use an assertion, or display
  // an error and quit).
  // - The fonts will be rasterized at a given size (w/ oversampling) and stored
  // into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which
  // ImGui_ImplXXXX_NewFrame below will call.
  // - Use '#define IMGUI_ENABLE_FREETYPE' in your imconfig file to use Freetype
  // for higher quality font rendering.
  // - Read 'docs/FONTS.md' for more instructions and details.
  // - Remember that in C/C++ if you want to include a backslash \ in a string
  // literal you need to write a double backslash \\ !
  // - Our Emscripten build process allows embedding fonts to be accessible at
  // runtime from the "fonts/" folder. See Makefile.emscripten for details.
  // io.Fonts->AddFontDefault();
  // io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\segoeui.ttf", 18.0f);
  // io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
  // io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
  // io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
  // ImFont* font =
  // io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f,
  // nullptr, io.Fonts->GetGlyphRangesJapanese()); IM_ASSERT(font != nullptr);

  // increase ImFontConfig::Density
  ImFontConfig config;
  config.SizePixels = 15.0f * 1.5f;
  config.OversampleH = 3;
  config.OversampleV = 3;
  config.PixelSnapH = true;

  // load Sans font
  io.Fonts->AddFontDefault();
  ImFont *font1 =
      io.Fonts->AddFontFromMemoryCompressedBase85TTF(SourceSans3Regular_compressed_data_base85, 24.0f, &config);
  IM_ASSERT(font1 != nullptr);

  // Our state
  // bool show_demo_window = true;
  // bool show_another_window = false;
  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

  std::thread computeThread(MyApp::run_calculation);

  // Render loop
  while (!glfwWindowShouldClose(window)) {
    // Poll and handle events (inputs, window resize, etc.)
    // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to
    // tell if dear imgui wants to use your inputs.
    // - When io.WantCaptureMouse is true, do not dispatch mouse input data to
    // your main application, or clear/overwrite your copy of the mouse data.
    // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input
    // data to your main application, or clear/overwrite your copy of the
    // keyboard data. Generally you may always pass all inputs to dear imgui,
    // and hide them from your application based on those two flags.
    glfwPollEvents();

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // My code
    ImGui::PushFont(font1);
    MyApp::RenderUI();
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
    // (Platform functions may change the current OpenGL context, so we
    // save/restore it to make it easier to paste this code elsewhere.
    //  For this specific demo app we could also call
    //  glfwMakeContextCurrent(window) directly)
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
      GLFWwindow *backup_current_context = glfwGetCurrentContext();
      ImGui::UpdatePlatformWindows();
      ImGui::RenderPlatformWindowsDefault();
      glfwMakeContextCurrent(backup_current_context);
    }

    glfwSwapBuffers(window);
  }

  // Cleanup
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  ImPlot::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();

  // Stop the compute thread
  {
    std::lock_guard<std::mutex> lock(MyApp::OpdMeasurementMutex);
    MyApp::RunOpdMeasurement.store(false);
  }
  computeThread.join();

  return 0;
}
