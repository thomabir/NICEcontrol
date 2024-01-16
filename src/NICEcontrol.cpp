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

// IIR filter from https://github.com/berndporr/iir1
#include "Iir.h"

// Fast Fourier Transform
#include <fftw3.h>

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
  ScrollingBuffer(int max_size = 70000) {
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

class FFT_calculator {
    public:
      int size;
      float sampling_rate;
      ScrollingBuffer *measurement_buffer;
      double *output_power;
      double *output_freq;
      fftw_complex *in, *out;
      fftw_plan fft_plan;


      FFT_calculator(int size, float sampling_rate, ScrollingBuffer *measurement_buffer, double *output_power, double *output_freq) {
        // initialise fftw
        in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * size);
        out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * size);
        fft_plan = fftw_plan_dft_1d(size, in, out, FFTW_FORWARD, FFTW_ESTIMATE);

        // initialise variables
        this->size = size;
        this->sampling_rate = sampling_rate;
        this->measurement_buffer = measurement_buffer;
        this->output_power = output_power;
        this->output_freq = output_freq;
      }

      void calculate() {
        // if there are at least size points in the measurement_buffer, copy the size latest points to in
        // Note: the most recent point is at measurement_buffer.Data[measurement_buffer.Offset].
        // While looping over the buffer, if you reach measurement_buffer[0], you need to continue at measurement_buffer.Data[measurement_buffer.MaxSize-1]
        int offset = measurement_buffer->Offset;
        static int j = 0;
        static int max_size = measurement_buffer->MaxSize;

        if (measurement_buffer->Data.size() == max_size) {
          for (int i = 0; i < size; i++) {
            in[i][0] = measurement_buffer->Data[(offset - size + i + max_size) % max_size].y;
            in[i][1] = 0.0f;
          }
        }

        // execute fft
        fftw_execute(fft_plan);

        // calculate power spectrum (only the real half)
        for (int i = 0; i < size / 2; i++) {
          output_power[i] = out[i][0] * out[i][0] + out[i][1] * out[i][1];
        }

        // set first element to 0 (DC)
        output_power[0] = 0.0f;

        // find the frequency axis, assuming a sampling rate of 6.4 kHz
        for (int i = 0; i < size / 2; i++) {
          output_freq[i] = i * sampling_rate / size;
        }
      }
    };

// iir filters
Iir::Butterworth::LowPass<4> x1d_lp_filter, x2d_lp_filter, opd_lp_filter, opd_control_lp_filter;
const float x1d_samplingrate = 6400.;
const float x1d_cutoff = 50.;
const float opd_samplingrate = 64000.;
const float opd_cutoff = 1000.;
const float opd_control_cutoff = 50.;


namespace NICEcontrol {

// OPD control
float opd_setpoint_gui = 0.001f;  // setpoint entered in GUI, may be out of range
std::atomic<float> opd_setpoint = 0.001f;      // setpoint used in calculation, clipped to valid range
std::atomic<bool> RunOpdControl(false);
std::mutex OpdControlMutex;
std::condition_variable OpdControlCV;
std::atomic<float> p = 0.0f;
std::atomic<float> i = 0.0f;

// variables that control the measurement thread
std::atomic<bool> RunMeasurement(false);
std::mutex MeasurementMutex;
std::condition_variable MeasurementCV;
std::ofstream outputFile;



// a class for the opd stage to make it easier to use
class MCL_OPDStage {
 public:
  MCL_OPDStage() {
    // print startup
    std::cout << "Initialising OPD stage" << std::endl;
    handle = MCL_InitHandle();

    // if handle is 0: stage is not connected.
    if (handle == 0) {
      std::cout << "OPD stage not connected" << std::endl;
      return;
    }

    // move to zero position
    this->move_to(0.0f);

    std::cout << "OPD stage initialised" << std::endl;
  }

  ~MCL_OPDStage() {
    // move to zero position
    this->move_to(0.0f);
    std::cout << "OPD stage closed" << std::endl;
  }

  void move_to(float setpoint) {
    // move to setpoint in µm + 20 µm, to give some headroom in both directions
    MCL_SingleWriteN(setpoint + 20., 3, handle);
  }

  double read() {
    return MCL_SingleReadN(3, handle) - 20.;
  }

 private:
  int handle;
};

// initialise opd stage
MCL_OPDStage opd_stage;
static float opd_open_loop_setpoint = 0.0f;


TSQueue<Measurement> opdQueue;
TSQueue<Measurement> x1Queue;
TSQueue<Measurement> x2Queue;
TSQueue<Measurement> i1Queue;
TSQueue<Measurement> i2Queue;
TSQueue<Measurement> x1dQueue;
TSQueue<Measurement> x2dQueue;

// void run_calculation() {
//   static float measurement = 0.0f;

//   // outputFile.open("data.csv");
//   // outputFile << "Time (ms),Measurement\n";

//   while (true) {
//     {
//       std::unique_lock<std::mutex> lock(MeasurementMutex);
//       MeasurementCV.wait(lock, [] { return RunMeasurement.load(); });
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
//     opdQueue.push({t, measurement});

//     // Write measurement and time to the CSV file
//     // outputFile << currentTime << "," << measurement.load() << "\n";

//     // wait 100 µs
//     std::this_thread::sleep_for(std::chrono::microseconds(100));
//   }
// }







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
  std::ofstream outputFile("adc-data.csv");
  if (!outputFile) {
    std::cerr << "Failed to open output file." << std::endl;
  }

  int count = 0;
  int prev = 0;
  int buffer_size = 1024;
  char buffer[buffer_size];

  // initialise filter
  x1d_lp_filter.setup(x1d_samplingrate, x1d_cutoff);
  x2d_lp_filter.setup(x1d_samplingrate, x1d_cutoff);
  opd_lp_filter.setup(opd_samplingrate, opd_cutoff);

  // outputFile << "Time (s), Counter, OPD measurement (m)\n";

  while (true) {
    {
      std::unique_lock<std::mutex> lock(
          MeasurementMutex);  // the only purpose of that lock is to run/stop measurement?
      MeasurementCV.wait(lock, [] { return RunMeasurement.load(); });
    }

    // read the measurement from the ethernet connection
    count++;

    // Receive data
    struct sockaddr_in clientAddr;
    socklen_t clientAddrLen = sizeof(clientAddr);
    int numBytes = recvfrom(sockfd, buffer, buffer_size, 0, (struct sockaddr *)&clientAddr, &clientAddrLen);

    auto t = getTime();

    // Check for errors
    if (numBytes < 0) {
      std::cerr << "Error receiving data." << std::endl;
      break;
    }

    // Convert received data to vector of 10 ints
    int receivedDataInt[6*10];
    std::memcpy(receivedDataInt, buffer, sizeof(int) * 6*10);

    // data comes in like this: 10 x (count, x1, x2, opd, i1, i2)

    static float adc1[10];
    static float adc2[10];
    static float adc3[10];
    static float adc4[10];
    static float adc5[10];
    int counter[10];

    for (int i = 0; i < 10; i++) {

      // get counter
      counter[i] = receivedDataInt[6 * i];
      adc1[i] = receivedDataInt[6 * i + 1]; // x1
      adc2[i] = receivedDataInt[6 * i + 2]; // x2
      adc3[i] = receivedDataInt[6 * i + 3]/1000.; // opd
      adc4[i] = receivedDataInt[6 * i + 4]; // i1
      adc5[i] = receivedDataInt[6 * i + 5]; // i2

      // convert from millidegree to rad
      // receivedData[i] = receivedDataInt[2*i+1] * 1e-3 * M_PI / 180.;

      // from rad to nm, assuming 633 nm  = 2 pi rad
      // receivedData[i] *= 633. / (2. * M_PI);

      // save to file: current time, counter, phase
      // outputFile << t << "," << counter[i] << "," << receivedData[i] << "\n";
      // outputFile << t << "," << counter[i] << "," << adc1[i] << "," << adc2[i] << "," << adc3[i] << "," << adc4[i] << "\n";
      
    }


    // filter opd by piping the 10 new measurements through the filter
    float opd;
    for (int i = 0; i < 10; i++) {
      opd = opd_lp_filter.filter(adc3[i]);
    }

    float opd_control_measurement;
    for (int i = 0; i < 10; i++) {
      opd_control_measurement = opd_control_lp_filter.filter(adc3[i]);
    }





  

    // get and filter x1d
    float x1d = adc1[0] / adc4[0] * 1.11e3;
    x1d = x1d_lp_filter.filter(x1d);

    float x2d = adc2[0] / adc5[0] * 1.11e3;
    x2d = x2d_lp_filter.filter(x2d);

    // enqueue measurement and time
    opdQueue.push({t, opd});
    x1Queue.push({t, adc1[0]});
    x2Queue.push({t, adc2[0]});
    i1Queue.push({t, adc4[0]});
    i2Queue.push({t, adc5[0]});
    x1dQueue.push({t, x1d});
    x2dQueue.push({t, x2d});

    // variables for control
    static float opd_error = 0.0f;
    static float opd_error_integral = 0.0f;
    static float opd_control_signal = 0.0f;

    // if RunOpdControl is true, calculate the control signal
    if (RunOpdControl.load()) {
      // calculate error
      opd_error = opd_setpoint.load() - opd_control_measurement;

      // calculate integral
      opd_error_integral += i.load() * opd_error;

      // calculate derivative
      // opd_error_derivative = opd_error - opd_error_prev;

      // calculate control signal
      opd_control_signal = p.load() * opd_error +  opd_error_integral;

      // actuate piezo using class interface (takes input in µm)
      opd_stage.move_to(opd_control_signal * 1e-3);

    }


    

    // wait 100 µs
    // std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}

// Presenter
void startMeasurement() {
  RunMeasurement.store(true);
  MeasurementCV.notify_one();
}

void stopMeasurement() { RunMeasurement.store(false); }

void RenderUI() {
  ImGui::Begin("NICE Control");

  ImGuiIO &io = ImGui::GetIO();

  static auto current_measurement = 0.f;
  if (!opdQueue.isempty()) {
    current_measurement = opdQueue.back().value;
  }

  // Start measurement
  static bool measure_button = true;
  ImGui::Checkbox("Run measurement", &measure_button);
  if (measure_button) {
    startMeasurement();
  } else {
    stopMeasurement();
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

    
    // if (control_opd) {
    //     RunOpdControl.store(true);
    //   } else {
    //     RunOpdControl.store(false);
    //   }

    // run control if "Closed loop" is selected
    if (loop_select == 2) {
      RunOpdControl.store(true);
    } else {
      RunOpdControl.store(false);
    }

    // move stage to open loop setpoint if "Open loop" is selected
    if (loop_select == 1) {
      opd_stage.move_to(opd_open_loop_setpoint);
    }



    // real time plot
    static ScrollingBuffer opd_buffer, setpoint_buffer;
    
    static float t_gui = 0;

     // if measurement is running, update gui time.
    if (RunMeasurement.load()) {
      t_gui = getTime();
    }


    // add the entire MeasurementQueue to the buffer
    if (!opdQueue.isempty()) {
      while (!opdQueue.isempty()) {
        auto m = opdQueue.pop();
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

    static ImVec4 fft_color     = ImVec4(1,1,0,1);
    static float  thickness = 1;

    if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1, 200 * io.FontGlobalScale))) {
      ImPlot::SetupAxes(nullptr, nullptr, xflags, yflags);
      ImPlot::SetupAxisLimits(ImAxis_X1, t_gui - history_length, t_gui, ImGuiCond_Always);
      ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
      ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
      ImPlot::SetNextLineStyle(fft_color, thickness);
      ImPlot::PlotLine("Measurement", &opd_buffer.Data[0].x, &opd_buffer.Data[0].y, opd_buffer.Data.size(), 0,
                       opd_buffer.Offset, 2 * sizeof(float));
      if (plot_setpoint){
      ImPlot::PlotLine("Setpoint", &setpoint_buffer.Data[0].x, &setpoint_buffer.Data[0].y, setpoint_buffer.Data.size(),
                       0, setpoint_buffer.Offset, 2 * sizeof(float));
      }
      ImPlot::EndPlot();
    }

    
     if (ImGui::TreeNode("OPD FFT")) {

    // set up fft
    const static int fft_size = 1024;
    static double fft_power[fft_size / 2];
    static double fft_freq[fft_size / 2];
    static FFT_calculator fft(fft_size, 6400., &opd_buffer, fft_power, fft_freq);

    // calculate fft
    fft.calculate();


    // plot fft_power vs fft_freq, with log scale on x and y axis
    static float  fft_thickness = 3;
    static ImPlotAxisFlags fft_xflags = ImPlotAxisFlags_None;
    static ImPlotAxisFlags fft_yflags = ImPlotAxisFlags_None; //ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
    if (ImPlot::BeginPlot("##FFT", ImVec2(-1, 300 * io.FontGlobalScale))) {
      ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
      ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
      ImPlot::SetupAxes(nullptr, nullptr, fft_xflags, fft_yflags);
      ImPlot::SetupAxisLimits(ImAxis_X1, 10, 3200);
      ImPlot::SetupAxisLimits(ImAxis_Y1, 0.1, 1e8);
      ImPlot::SetNextLineStyle(fft_color, fft_thickness);
      ImPlot::PlotLine("FFT", &fft_freq[0], &fft_power[0], fft_size/2);
      ImPlot::EndPlot();
    }

    ImGui::TreePop();
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

      // plot for current piezo position
    static ImVec4 color     = ImVec4(1,1,0,1);
    static ScrollingBuffer piezo_buffer;
    piezo_buffer.AddPoint(t_gui, opd_stage.read());

    if (ImPlot::BeginPlot("##Piezo", ImVec2(-1, 150 * io.FontGlobalScale))) {
      ImPlot::SetupAxes(nullptr, nullptr, xflags, yflags);
      ImPlot::SetupAxisLimits(ImAxis_X1, t_gui - history_length, t_gui, ImGuiCond_Always);
      ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
      ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
      ImPlot::SetNextLineStyle(color, thickness);
      ImPlot::PlotLine("Piezo position", &piezo_buffer.Data[0].x, &piezo_buffer.Data[0].y, piezo_buffer.Data.size(), 0,
                       piezo_buffer.Offset, 2 * sizeof(float));
      ImPlot::EndPlot();
    }

      float piezo_measurement = 0.0f;

      // Display measurement
      ImGui::Text("Current measurement: %.4f", piezo_measurement);

      // open loop setpoint µm
      ImGui::SliderFloat("Setpoint", &opd_open_loop_setpoint, -20.f, 30.0f);

      ImGui::TreePop();
    }

    if (ImGui::TreeNode("Control loop")) {
      ImGui::Text("Control loop parameters:");

      // sliders for p , i
      static float p_gui = 0.125f;
      static float i_gui = 0.007f;
      
      ImGui::SliderFloat("P", &p_gui, 0.0f, 1.0f);
      ImGui::SliderFloat("I", &i_gui, 0.0f, 3e-2f);

      // store p, i
      p.store(p_gui);
      i.store(i_gui);

      const float opd_setpoint_min = -1000.0f, opd_setpoint_max = 1000.0f;
      
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

  if (ImGui::CollapsingHeader("X position")) {

    // real time plot
    static ScrollingBuffer x1_buffer;
    static ScrollingBuffer x2_buffer;
    static ScrollingBuffer i1_buffer;
    static ScrollingBuffer i2_buffer;
    static ScrollingBuffer x1d_buffer;
    static ScrollingBuffer x2d_buffer;

   
    static float t_gui_x = 0;

     // if measurement is running, update gui time.
    if (RunMeasurement.load()) {
      t_gui_x = getTime();
    }


    // add the entire MeasurementQueue to the buffer
    if (!x1Queue.isempty()) {
      while (!x1Queue.isempty()) {
        auto m = x1Queue.pop();
        x1_buffer.AddPoint(m.time, m.value);
      }
    }

    if (!x2Queue.isempty()) {
      while (!x2Queue.isempty()) {
        auto m = x2Queue.pop();
        x2_buffer.AddPoint(m.time, m.value);
      }
    }

    if (!i1Queue.isempty()) {
      while (!i1Queue.isempty()) {
        auto m = i1Queue.pop();
        i1_buffer.AddPoint(m.time, m.value);
      }
    }

    if (!i2Queue.isempty()) {
      while (!i2Queue.isempty()) {
        auto m = i2Queue.pop();
        i2_buffer.AddPoint(m.time, m.value);
      }
    }

    if (!x1dQueue.isempty()) {
      while (!x1dQueue.isempty()) {
        auto m = x1dQueue.pop();
        x1d_buffer.AddPoint(m.time, m.value);
      }
    }

    if (!x2dQueue.isempty()) {
      while (!x2dQueue.isempty()) {
        auto m = x2dQueue.pop();
        x2d_buffer.AddPoint(m.time, m.value);
      }
    }

    static float x1_history_length = 10.0f;
    ImGui::SliderFloat("History", &x1_history_length, 0.1, 10, "%.2f s", ImGuiSliderFlags_Logarithmic);

    // x axis: no ticks
    static ImPlotAxisFlags x1_xflags = ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoTickLabels;

    // y axis: auto fit
    static ImPlotAxisFlags x1_yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;

    static ImVec4 x1_color     = ImVec4(1,1,0,1);
    static ImVec4 x2_color     = ImVec4(1,0,0,1);
    static ImVec4 i1_color     = ImVec4(0,1,0,1);
    static ImVec4 i2_color     = ImVec4(0,0,1,1);
    static float  x1_thickness = 1;

    // To debug x1, x2, i1, i2
    // if (ImPlot::BeginPlot("##X1_Scrolling", ImVec2(-1, 200 * io.FontGlobalScale))) {
    //   ImPlot::SetupAxes(nullptr, nullptr, x1_xflags, x1_yflags);
    //   ImPlot::SetupAxisLimits(ImAxis_X1, t_gui_x - x1_history_length, t_gui_x, ImGuiCond_Always);
    //   ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
    //   ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
    //   ImPlot::SetNextLineStyle(x1_color, x1_thickness);
    //   ImPlot::PlotLine("x1", &x1_buffer.Data[0].x, &x1_buffer.Data[0].y, x1_buffer.Data.size(), 0,
    //                    x1_buffer.Offset, 2 * sizeof(float));
    //   ImPlot::SetNextLineStyle(x2_color, x1_thickness);
    //   ImPlot::PlotLine("x2", &x2_buffer.Data[0].x, &x2_buffer.Data[0].y, x2_buffer.Data.size(), 0,
    //                    x2_buffer.Offset, 2 * sizeof(float));
    //   ImPlot::SetNextLineStyle(i1_color, x1_thickness);
    //   ImPlot::PlotLine("i1", &i1_buffer.Data[0].x, &i1_buffer.Data[0].y, i1_buffer.Data.size(), 0,
    //                    i1_buffer.Offset, 2 * sizeof(float));
    //   ImPlot::SetNextLineStyle(i2_color, x1_thickness);
    //   ImPlot::PlotLine("i2", &i2_buffer.Data[0].x, &i2_buffer.Data[0].y, i2_buffer.Data.size(), 0,
    //                    i2_buffer.Offset, 2 * sizeof(float));
    //   ImPlot::EndPlot();
    // }


    // plot x1d, x2d
    if (ImPlot::BeginPlot("##X1D", ImVec2(-1, 200 * io.FontGlobalScale))) {
      ImPlot::SetupAxes(nullptr, nullptr, x1_xflags, x1_yflags);
      ImPlot::SetupAxisLimits(ImAxis_X1, t_gui_x - x1_history_length, t_gui_x, ImGuiCond_Always);
      ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
      ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
      ImPlot::SetNextLineStyle(x1_color, x1_thickness);
      ImPlot::PlotLine("x1d", &x1d_buffer.Data[0].x, &x1d_buffer.Data[0].y, x1d_buffer.Data.size(), 0,
                       x1d_buffer.Offset, 2 * sizeof(float));
      ImPlot::SetNextLineStyle(x2_color, x1_thickness);
      ImPlot::PlotLine("x2d", &x2d_buffer.Data[0].x, &x2d_buffer.Data[0].y, x2d_buffer.Data.size(), 0,
                       x2d_buffer.Offset, 2 * sizeof(float));
      ImPlot::EndPlot();
    }

    // calculate std and rms of x1d over last 1000 points
    static float x1d_std = 0.0f;
    static float x1d_rms = 0.0f;
    static float x2d_std = 0.0f;
    static float x2d_rms = 0.0f;

    if (x1d_buffer.Data.size() > 0) {
      // calculate mean
      float sum = 0.0f;
      for (auto &p : x1d_buffer.Data) {
        sum += p.y;
      }
      float mean = sum / x1d_buffer.Data.size();

      // calculate std
      float sum_sq = 0.0f;
      for (auto &p : x1d_buffer.Data) {
        sum_sq += (p.y - mean) * (p.y - mean);
      }
      x1d_std = sqrt(sum_sq / x1d_buffer.Data.size());

      // calculate rms
      sum_sq = 0.0f;
      for (auto &p : x1d_buffer.Data) {
        sum_sq += p.y * p.y;
      }
      x1d_rms = sqrt(sum_sq / x1d_buffer.Data.size());
    }

  // print it
  ImGui::Text("x1d std: %.4f", x1d_std);
  ImGui::Text("x1d rms: %.4f", x1d_rms);

  
  // FFT
  if (ImGui::TreeNode("X1D FFT")) {

    // set up fft of x1d
    const static int fft_size = 1024*8*8;
    static double fft_power[fft_size / 2];
    static double fft_freq[fft_size / 2];
    static FFT_calculator fft(fft_size, 6400., &x1d_buffer, fft_power, fft_freq);

    // calculate fft
    fft.calculate();


      static float  fft_thickness = 3;
      ImVec4 fft_color = ImVec4(1,1,0,1);

      // x axis: no flags
      static ImPlotAxisFlags fft_xflags = ImPlotAxisFlags_None;
      static ImPlotAxisFlags fft_yflags = ImPlotAxisFlags_None; //ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
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
        ImPlot::PlotLine("FFT", &fft_freq[0], &fft_power[0], fft_size/2);
        ImPlot::EndPlot();
      }
  ImGui::TreePop();
  }

  


    }



  if (ImGui::CollapsingHeader("Program settings")) {
    ImGui::DragFloat("GUI scale", &io.FontGlobalScale, 0.005f, 0.5, 3.0, "%.2f",
                     ImGuiSliderFlags_AlwaysClamp);  // Scale everything
  }
  ImGui::End();

  // demo window
  // ImGui::ShowDemoWindow();

  // implot demo window
  // ImPlot::ShowDemoWindow();
}

}  // namespace NICEcontrol

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

  std::thread computeThread(NICEcontrol::run_calculation);

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
    NICEcontrol::RenderUI();
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
    std::lock_guard<std::mutex> lock(NICEcontrol::MeasurementMutex);
    NICEcontrol::RunMeasurement.store(false);
  }
  computeThread.join();

  return 0;
}
