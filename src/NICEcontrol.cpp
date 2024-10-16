#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h>  // Will drag system OpenGL headers
#include <stdio.h>

#include <array>
#include <atomic>
#include <boost/circular_buffer.hpp>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <mutex>
#include <queue>
#include <random>
#include <thread>

// ethernet
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

// IIR filter from https://github.com/berndporr/iir1
#include "Iir.h"

// Fast Fourier Transform
#include <fftw3.h>

#include <cstring>

#include "../lib/fonts/SourceSans3Regular.cpp"
#include "../lib/implot/implot.h"

// Actuators
#include "MCL_NanoDrive.hpp"       // Controller for MCL OPD Stage
#include "PI_E727_Controller.hpp"  // Controller for PI Tip/Tilt Stages
#include "PI_E754_Controller.hpp"  // Controller for PI OPD Stage
#include "nF_EBD_Controller.hpp"   // Controller for nanoFaktur Tip/Tilt Stages

// Controllers (PID etc.)
#include "Controllers.hpp"

// NullLockin
#include "NullLockin.hpp"

// white noise for dithering
#include <random>

// format datetime for logging
#include <ctime>

// Windows
#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

// Constants
#define PI 3.14159265359

static void glfw_error_callback(int error, const char *description) {
  fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

double getTime() {
  // returns time in seconds since the start of the program
  static auto t0 = std::chrono::system_clock::now();
  auto tnow = std::chrono::system_clock::now();
  double t_since_start = std::chrono::duration_cast<std::chrono::microseconds>(tnow - t0).count() / 1.0e6;
  return t_since_start;
}

std::string get_iso_datestring() {
  time_t now;
  time(&now);
  char buf[sizeof "2011-10-08T07:07:09Z"];
  strftime(buf, sizeof buf, "%FT%TZ", gmtime(&now));
  // this will work too, if your compiler doesn't support %F or %T:
  // strftime(buf, sizeof buf, "%Y-%m-%dT%H:%M:%SZ", gmtime(&now));

  // replace all colons with dashes
  std::string str(buf);
  std::replace(str.begin(), str.end(), ':', '-');
  return str;
}

struct Measurement {
  double time;
  float value;
};

class ControlData {
 public:
  double time;
  float measurement;
  float setpoint;
  float dither_signal;
  float controller_input;
  float controller_output;
  float actuator_command;

  ControlData(double time, float measurement, float setpoint, float dither_signal, float controller_input,
              float controller_output, float actuator_command) {
    this->time = time;
    this->measurement = measurement;
    this->setpoint = setpoint;
    this->dither_signal = dither_signal;
    this->controller_input = controller_input;
    this->controller_output = controller_output;
    this->actuator_command = actuator_command;
  }

  ControlData() {
    this->time = 0.0;
    this->measurement = 0.0;
    this->setpoint = 0.0;
    this->dither_signal = 0.0;
    this->controller_input = 0.0;
    this->controller_output = 0.0;
    this->actuator_command = 0.0;
  }

  ControlData(const ControlData &other) {
    this->time = other.time;
    this->measurement = other.measurement;
    this->setpoint = other.setpoint;
    this->dither_signal = other.dither_signal;
    this->controller_input = other.controller_input;
    this->controller_output = other.controller_output;
    this->actuator_command = other.actuator_command;
  }

  ControlData &operator=(const ControlData &other) {
    this->time = other.time;
    this->measurement = other.measurement;
    this->setpoint = other.setpoint;
    this->dither_signal = other.dither_signal;
    this->controller_input = other.controller_input;
    this->controller_output = other.controller_output;
    this->actuator_command = other.actuator_command;
    return *this;
  }
};

class SensorData {
 public:
  double time;
  float opd;
  float shear_x1;
  float shear_x2;
  float shear_y1;
  float shear_y2;
  float point_x1;
  float point_x2;
  float point_y1;
  float point_y2;

  SensorData(double time, float opd, float shear_x1, float shear_x2, float shear_y1, float shear_y2, float point_x1,
             float point_x2, float point_y1, float point_y2) {
    this->time = time;
    this->opd = opd;
    this->shear_x1 = shear_x1;
    this->shear_x2 = shear_x2;
    this->shear_y1 = shear_y1;
    this->shear_y2 = shear_y2;
    this->point_x1 = point_x1;
    this->point_x2 = point_x2;
    this->point_y1 = point_y1;
    this->point_y2 = point_y2;
  }

  SensorData() {
    this->time = 0.0;
    this->opd = 0.0;
    this->shear_x1 = 0.0;
    this->shear_x2 = 0.0;
    this->shear_y1 = 0.0;
    this->shear_y2 = 0.0;
    this->point_x1 = 0.0;
    this->point_x2 = 0.0;
    this->point_y1 = 0.0;
    this->point_y2 = 0.0;
  }

  SensorData(const SensorData &other) {
    this->time = other.time;
    this->opd = other.opd;
    this->shear_x1 = other.shear_x1;
    this->shear_x2 = other.shear_x2;
    this->shear_y1 = other.shear_y1;
    this->shear_y2 = other.shear_y2;
    this->point_x1 = other.point_x1;
    this->point_x2 = other.point_x2;
    this->point_y1 = other.point_y1;
    this->point_y2 = other.point_y2;
  }

  SensorData &operator=(const SensorData &other) {
    this->time = other.time;
    this->opd = other.opd;
    this->shear_x1 = other.shear_x1;
    this->shear_x2 = other.shear_x2;
    this->shear_y1 = other.shear_y1;
    this->shear_y2 = other.shear_y2;
    this->point_x1 = other.point_x1;
    this->point_x2 = other.point_x2;
    this->point_y1 = other.point_y1;
    this->point_y2 = other.point_y2;
    return *this;
  }

  // acces by index
  float &operator[](int i) {
    switch (i) {
      case 0:
        return opd;
      case 1:
        return shear_x1;
      case 2:
        return shear_x2;
      case 3:
        return shear_y1;
      case 4:
        return shear_y2;
      case 5:
        return point_x1;
      case 6:
        return point_x2;
      case 7:
        return point_y1;
      case 8:
        return point_y2;
      default:
        return opd;
    }
  }
};

template <typename T, typename U>
struct MeasurementT {
  T time;
  U value;
};

// thread-safe circular buffer
template <typename T>
class TSCircularBuffer {
 private:
  boost::circular_buffer<T> m_buffer;
  std::mutex m_mutex;

 public:
  TSCircularBuffer() : m_buffer(1000000) {}       // default size: 1e6
  TSCircularBuffer(int size) : m_buffer(size) {}  // custom size

  void push(T item) {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_buffer.push_back(item);
  }

  T front() {
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_buffer.front();
  }

  T back() {
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_buffer.back();
  }

  T pop() {
    std::unique_lock<std::mutex> lock(m_mutex);
    T item = m_buffer.front();
    m_buffer.pop_front();
    return item;
  }

  bool isempty() {
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_buffer.empty();
  }

  int size() {
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_buffer.size();
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
    // fill with zeros
    for (int i = 0; i < MaxSize; i++) {
      Data.push_back(ImVec2(0, 0));
    }
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

  // get last n points
  ImVector<ImVec2> GetLastN(int n) {
    ImVector<ImVec2> last_n;
    last_n.reserve(n);
    for (int i = 0; i < n; i++) {
      last_n.push_back(Data[(Offset - n + i + MaxSize) % MaxSize]);
    }
    return last_n;
  }
};

// scrolling buffer for data of type MeasurementT
// does not use ImVector<ImVec2> but ImVector<MesurementT>
template <typename T, typename U>
struct ScrollingBufferT {
  int MaxSize;
  int Offset;
  ImVector<MeasurementT<T, U>> Data;
  ScrollingBufferT(int max_size = 70000) {
    MaxSize = max_size;
    Offset = 0;
    Data.reserve(MaxSize);
    // fill with zeros
    for (int i = 0; i < MaxSize; i++) {
      Data.push_back(MeasurementT<T, U>(0, 0));
    }
  }
  void AddPoint(T x, U y) {
    if (Data.size() < MaxSize)
      Data.push_back(MeasurementT<T, U>(x, y));
    else {
      Data[Offset] = MeasurementT<T, U>(x, y);
      Offset = (Offset + 1) % MaxSize;
    }
  }
  void Erase() {
    if (Data.size() > 0) {
      Data.shrink(0);
      Offset = 0;
    }
  }

  // get last n points
  ImVector<MeasurementT<T, U>> GetLastN(int n) {
    ImVector<MeasurementT<T, U>> last_n;
    last_n.reserve(n);
    for (int i = 0; i < n; i++) {
      last_n.push_back(Data[(Offset - n + i + MaxSize) % MaxSize]);
    }
    return last_n;
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

  FFT_calculator(int size, float sampling_rate, ScrollingBuffer *measurement_buffer, double *output_power,
                 double *output_freq) {
    // initialise fftw
    in = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * size);
    out = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * size);
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
    // While looping over the buffer, if you reach measurement_buffer[0], you need to continue at
    // measurement_buffer.Data[measurement_buffer.MaxSize-1]
    int offset = measurement_buffer->Offset;
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

// Filters for sensor data
Iir::Butterworth::LowPass<2> shear_x1_lpfilt, shear_x2_lpfilt, shear_y1_lpfilt, shear_y2_lpfilt;
Iir::Butterworth::LowPass<2> point_x1_lpfilt, point_x2_lpfilt, point_y1_lpfilt, point_y2_lpfilt;
Iir::Butterworth::LowPass<2> opd_lp_filter;
const float shear_samplingrate = 128000.;
const float shear_lpfilt_cutoff = 300.;
const float opd_samplingrate = 128000.;
const float opd_lpfilt_cutoff = 1000.;

namespace NICEcontrol {

// general
std::atomic<bool> gui_control(true);

// OPD control
float opd_setpoint_gui = 0.0f;  // setpoint entered in GUI, may be out of range

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

// variables that control the measurement thread
std::atomic<bool> RunMeasurement(false);

// initialise opd stage
// MCL_OPDStage opd_stage;
char serial_number[1024] = "123076463";
PI_E754_Controller opd_stage(serial_number);

// initialise tip/tilt stages
char serial_number1[1024] = "0122040101";
PI_E727_Controller tip_tilt_stage1(serial_number1);

char serial_number2[1024] = "0122042007";
PI_E727_Controller tip_tilt_stage2(serial_number2);

nF_EBD_Controller nF_stage_1("/dev/ttyUSB2");
nF_EBD_Controller nF_stage_2("/dev/ttyUSB3");

void setupActuators() {
  // connect and intialise all piezo stages

  // Tip/tilt stage 1
  tip_tilt_stage1.init();
  // tip_tilt_stage1.autozero(); // run autozero if stage does not move
  tip_tilt_stage1.move_to_x(0.0f);
  tip_tilt_stage1.move_to_y(0.0f);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << "\tPosition: (" << tip_tilt_stage1.readx() << ", " << tip_tilt_stage1.ready() << ") urad" << std::endl;

  // Tip/tilt stage 2
  tip_tilt_stage2.init();
  // tip_tilt_stage2.autozero(); // run autozero if stage does not move
  tip_tilt_stage2.move_to_x(0.0f);
  tip_tilt_stage2.move_to_y(0.0f);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << "\tPosition: (" << tip_tilt_stage2.readx() << ", " << tip_tilt_stage2.ready() << ") urad" << std::endl;

  // nF tip/tilt stages
  nF_stage_1.init();
  nF_stage_1.move_to({0.0, 0.0});
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto pos = nF_stage_1.read();
  std::cout << "nF Stage 1 Position: " << pos[0] << ", " << pos[1] << std::endl;

  nF_stage_2.init();
  nF_stage_2.move_to({0.0, 0.0});
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  pos = nF_stage_2.read();
  std::cout << "nF Stage 2 Position: " << pos[0] << ", " << pos[1] << std::endl;

  // OPD stage
  opd_stage.init();
  opd_stage.move_to(0.0f);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << "\tPosition: " << opd_stage.read() << " nm" << std::endl;
}

TSCircularBuffer<SensorData> sensorDataQueue;

TSCircularBuffer<MeasurementT<int, int>> adc_queues[10];
TSCircularBuffer<MeasurementT<int, int>> shear_sum_queue, point_sum_queue;
TSCircularBuffer<MeasurementT<int, int>> adc_sci_null_queue, adc_sci_mod_queue;
TSCircularBuffer<MeasurementT<double, double>> sci_null_queue;

int setup_ethernet() {
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

  return sockfd;
}

// N PI controllers, each with P and I gains, diagonal N x N system
template <int N>
class DiagPIController {
 public:
  DiagPIController() {
    Ps.fill(0.);
    Is.fill(0.);
    error_integrals.fill(0.);
  }

  void setPI(std::array<float, N> Ps, std::array<float, N> Is) {
    this->Ps = Ps;
    this->Is = Is;
  }

  void reset_state() { error_integrals.fill(0); }

  void reset_all() {
    error_integrals.fill(0);
    Ps.fill(0);
    Is.fill(0);
  }

  std::array<float, N> step(std::array<float, N> inputs) {
    std::array<float, N> outputs;
    for (int i = 0; i < N; i++) {
      error_integrals[i] += inputs[i];
      outputs[i] = Ps[i] * inputs[i] + Is[i] * error_integrals[i];
    }
    return outputs;
  }

 private:
  std::array<float, N> Ps;
  std::array<float, N> Is;
  std::array<float, N> error_integrals;
};

template <int N, class C, class A>  // N x N MIMO control loop
class MIMOControlLoop {
 public:
  // control mode is 0 by default
  MIMOControlLoop(C &controller, A &actuator) : controller(controller), actuator(actuator) {}

  void control(double t, std::array<float, N> sensor_data) {
    std::array<float, N> controller_input = {0.0f};
    std::array<float, N> controller_output = {0.0f};
    std::array<float, N> actuator_command = {0.0f};

    // load all atomic variables into non-atomic ones to avoid changes while this function is running
    std::array<float, N> setpoint_loc = this->setpoint.load();
    std::array<float, N> Ps_loc = this->Ps.load();
    std::array<float, N> Is_loc = this->Is.load();
    float dither_freq_loc = this->dither_freq.load();
    float dither_amp_loc = this->dither_amp.load();
    int dither_axis_loc = this->dither_axis.load();
    std::array<int, N> dither_mode_loc = this->dither_mode.load();
    std::array<int, N> plant_mode_loc = this->plant_mode.load();
    std::array<int, N> controller_mode_loc = this->controller_mode.load();

    // calculate dither signal: zero except for the dither axis
    std::array<float, N> dither_signal = {0.0f};
    dither_signal[dither_axis_loc] = dither_amp_loc * std::sin(2 * PI * dither_freq_loc * t);

    // controller: set p and i
    this->controller.setPI(Ps_loc, Is_loc);

    // assemble controller input
    for (int i = 0; i < N; i++) {
      switch (controller_mode_loc[i]) {
        case 0:  // 0 as input
          controller_input[i] = 0.0f;
          break;
        case 1:  // setpoint - measurement as input
          controller_input[i] = setpoint_loc[i] - sensor_data[i];
          break;
        default:
          // don't
          break;
      }
      if (dither_mode_loc[i] == 1) {
        controller_input[i] += dither_signal[i];
      }
    }

    // calculate controller output
    controller_output = this->controller.step(controller_input);

    // calculate actuator command
    for (int i = 0; i < N; i++) {
      switch (plant_mode_loc[i]) {
        case 0:  // off
          break;
        case 1:  // plant gets setpoint
          actuator_command[i] = setpoint_loc[i];
          break;
        case 2:  // plant gets controller output
          actuator_command[i] = controller_output[i];
          break;
        default:
          // don't
          break;
      }
      if (dither_mode_loc[i] == 2) {
        actuator_command[i] += dither_signal[i];
      }
    }

    // move actuator
    actuator.move_to(actuator_command);

    // prepare data to be pushed to the buffer
    // make an array of N controldata objects
    // ControlData control_data[N];

    // fill the array with data
    // for (int i = 0; i < N; i++) {
    //   control_data[i] = {t,
    //                      sensor_data[i],
    //                      setpoint[i],
    //                      dither_signal[i],
    //                      controller_input[i],
    //                      controller_output[i],
    //                      actuator_command[i]};
    // }

    // push the data to the buffer
    // this->controlDataBuffer.push(control_data);
  }

  std::atomic<std::array<float, N>> setpoint{};
  std::atomic<std::array<float, N>> Ps{};
  std::atomic<std::array<float, N>> Is{};
  std::atomic<float> dither_freq = 0.0f;
  std::atomic<float> dither_amp = 0.0f;
  std::atomic<int> dither_axis = 0;
  std::atomic<std::array<int, N>> dither_mode{};  // 0 = off, 1 = dither controller, 2 = dither plant
  std::atomic<std::array<int, N>> plant_mode{};   // 0 = off, 1 = plant gets setpoint, 2 = plant gets controller output
  std::atomic<std::array<int, N>> controller_mode{};  // 0 = 0 as input, 1 = setpoint - measurement as input

  A &actuator;
  C &controller;

  // data buffer: N x controldata
  // TSCircularBuffer<ControlDataN<N>> controlDataBuffer;
};

template <class C, class A>  // SISO control loop
class SISOControlLoop {
 public:
  // control mode is 0 by default
  SISOControlLoop(C &controller, A &stage) : controller(controller), stage(stage) {}

  void reset_all() {
    // reset loop
    this->control_mode.store(0);
    this->setpoint.store(0.0f);
    this->p.store(0.0f);
    this->i.store(0.0f);
    this->dither_freq.store(0.0f);
    this->dither_amp.store(0.0f);

    // reset controller
    this->controller.reset_all();  // also resets P, I
  }

  void control(double t, float measurement) {
    // OPD control
    float controller_input = 0.0f;
    float controller_output = 0.0f;
    float actuator_command = 0.0f;

    // calculate dither signal
    float dither_signal = this->dither_amp.load() * std::sin(2 * PI * this->dither_freq.load() * t);

    // get control parameters
    float setpoint = this->setpoint.load();
    this->controller.setPI(this->p.load(), this->i.load());
    int cm = this->control_mode.load();

    // if control_mode has changed, reset the controller
    // static int prev_cm = cm;
    // if (cm != prev_cm) {
    //   this->controller.reset_state();
    // }
    // prev_cm = cm;

    switch (cm) {
      case 0:  // do nothing
        break;
      case 1:  // P
        actuator_command = (setpoint + dither_signal);
        this->stage.move_to(actuator_command);
        break;
      case 2:  // C
        controller_input = dither_signal;
        controller_output = this->controller.step(controller_input);
        break;
      case 3:  // CP open
        controller_input = dither_signal;
        controller_output = this->controller.step(controller_input);
        actuator_command = controller_output;
        this->stage.move_to(actuator_command);
        break;
      case 4:  // CP closed, dither plant
        controller_input = setpoint - measurement;
        controller_output = this->controller.step(controller_input);
        actuator_command = controller_output + dither_signal;
        this->stage.move_to(actuator_command);  // convert nm to um
        break;
      case 5:  // CP closed, dither setpoint
        controller_input = setpoint + dither_signal - measurement;
        controller_output = this->controller.step(controller_input);
        actuator_command = controller_output;
        this->stage.move_to(actuator_command);  // convert nm to um
        break;
      default:
        // don't
        break;
    }

    this->controlDataBuffer.push(
        {t, measurement, setpoint, dither_signal, controller_input, controller_output, actuator_command});
  }

  std::atomic<int> control_mode = 0;
  std::atomic<float> setpoint = 0.0f;
  std::atomic<float> p = 0.0f;
  std::atomic<float> i = 0.0f;
  std::atomic<float> dither_freq = 0.0f;
  std::atomic<float> dither_amp = 0.0f;
  C &controller;
  A &stage;
  TSCircularBuffer<ControlData> controlDataBuffer;
};

class ShearXActuator {
 public:
  ShearXActuator(PI_E727_Controller &stage) : stage(stage) {}

  void move_to(float position) { stage.move_to_x(position); }

 private:
  PI_E727_Controller &stage;
};

class ShearYActuator {
 public:
  ShearYActuator(PI_E727_Controller &stage) : stage(stage) {}

  void move_to(float position) { stage.move_to_y(position); }

 private:
  PI_E727_Controller &stage;
};

// controllers
PIController opd_controller;
PIController shear_x1_controller;
PIController shear_x2_controller;
PIController shear_y1_controller;
PIController shear_y2_controller;
DiagPIController<2> point_1_controller;
DiagPIController<2> point_2_controller;

// actuators
ShearXActuator shear_x1_actuator(tip_tilt_stage1);
ShearXActuator shear_x2_actuator(tip_tilt_stage2);
ShearYActuator shear_y1_actuator(tip_tilt_stage1);
ShearYActuator shear_y2_actuator(tip_tilt_stage2);

// control loops
SISOControlLoop opd_loop(opd_controller, opd_stage);
SISOControlLoop shear_x1_loop(shear_x1_controller, shear_x1_actuator);
SISOControlLoop shear_x2_loop(shear_x2_controller, shear_x2_actuator);
SISOControlLoop shear_y1_loop(shear_y1_controller, shear_y1_actuator);
SISOControlLoop shear_y2_loop(shear_y2_controller, shear_y2_actuator);
MIMOControlLoop<2, DiagPIController<2>, nF_EBD_Controller> point_1_loop(point_1_controller, nF_stage_1);
MIMOControlLoop<2, DiagPIController<2>, nF_EBD_Controller> point_2_loop(point_2_controller, nF_stage_2);
NullLockin null_lockin;

void run_calculation() {
  int sockfd = setup_ethernet();

  int count = 0;
  int buffer_size = 1024;
  char buffer[buffer_size];

  // setup filters
  shear_x1_lpfilt.setup(shear_samplingrate, shear_lpfilt_cutoff);
  shear_x2_lpfilt.setup(shear_samplingrate, shear_lpfilt_cutoff);
  shear_y1_lpfilt.setup(shear_samplingrate, shear_lpfilt_cutoff);
  shear_y2_lpfilt.setup(shear_samplingrate, shear_lpfilt_cutoff);

  point_x1_lpfilt.setup(shear_samplingrate, shear_lpfilt_cutoff);
  point_x2_lpfilt.setup(shear_samplingrate, shear_lpfilt_cutoff);
  point_y1_lpfilt.setup(shear_samplingrate, shear_lpfilt_cutoff);
  point_y2_lpfilt.setup(shear_samplingrate, shear_lpfilt_cutoff);

  opd_lp_filter.setup(opd_samplingrate, opd_lpfilt_cutoff);

  // initialise null processir

  while (true) {
    float opd_rad_f = 0., shear_x1_f = 0., shear_x2_f = 0., shear_y1_f = 0., shear_y2_f = 0., point_x1_f = 0.,
          point_x2_f = 0., point_y1_f = 0., point_y2_f = 0., opd_nm_f = 0., sci_null = 0.;
    auto t = getTime();
    auto t_chrono = std::chrono::high_resolution_clock::now();

    RunMeasurement.wait(false);

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
    const int num_channels = 22;
    const int num_timepoints = 10;
    int receivedDataInt[num_channels * num_timepoints];
    std::memcpy(receivedDataInt, buffer, sizeof(int) * num_channels * num_timepoints);

    static int counter[10];
    static int adc_shear1[10];
    static int adc_shear2[10];
    static int adc_shear3[10];
    static int adc_shear4[10];
    static int adc_point1[10];
    static int adc_point2[10];
    static int adc_point3[10];
    static int adc_point4[10];
    static int adc_sine_ref[10];
    static int adc_sci_null[10];
    static int adc_sci_mod[10];
    static int adc_opd_ref[10];
    static float opd_rad[10];
    static int32_t opd_int[10];
    static float shear_x1_um[10];
    static float shear_x2_um[10];
    static float shear_y1_um[10];
    static float shear_y2_um[10];
    static float point_x1_um[10];
    static float point_x2_um[10];
    static float point_y1_um[10];
    static float point_y2_um[10];
    static float opd_rad_i_prev = 0.0f;

    for (int i = 0; i < num_timepoints; i++) {
      counter[i] = receivedDataInt[num_channels * i];
      adc_shear1[i] = receivedDataInt[num_channels * i + 1];
      adc_shear2[i] = receivedDataInt[num_channels * i + 2];
      adc_shear3[i] = receivedDataInt[num_channels * i + 3];
      adc_shear4[i] = receivedDataInt[num_channels * i + 4];
      adc_point1[i] = receivedDataInt[num_channels * i + 5];
      adc_point2[i] = receivedDataInt[num_channels * i + 6];
      adc_point3[i] = receivedDataInt[num_channels * i + 7];
      adc_point4[i] = receivedDataInt[num_channels * i + 8];
      adc_sine_ref[i] = receivedDataInt[num_channels * i + 9];
      adc_opd_ref[i] = receivedDataInt[num_channels * i + 10];
      opd_int[i] = receivedDataInt[num_channels * i + 11];
      opd_rad[i] = -float(opd_int[i]) * PI / (std::pow(2.0, 23) - 1.);         // phase (signed 24 bit int) -> rad
      shear_x1_um[i] = float(receivedDataInt[num_channels * i + 12]) / 3000.;  // um
      shear_x2_um[i] = float(receivedDataInt[num_channels * i + 13]) / 3000.;  // um
      shear_y1_um[i] = float(receivedDataInt[num_channels * i + 14]) / 3000.;  // um
      shear_y2_um[i] = float(receivedDataInt[num_channels * i + 15]) / 3000.;  // um
      point_x1_um[i] = float(receivedDataInt[num_channels * i + 16]) / 1000.;  // urad
      point_x2_um[i] = float(receivedDataInt[num_channels * i + 17]) / 1000.;  // urad
      point_y1_um[i] = float(receivedDataInt[num_channels * i + 18]) / 1000.;  // urad
      point_y2_um[i] = float(receivedDataInt[num_channels * i + 19]) / 1000.;  // urad
      adc_sci_null[i] = receivedDataInt[num_channels * i + 20];                // ADU
      adc_sci_mod[i] = receivedDataInt[num_channels * i + 21];                 // ADU
    }

    // phase-unwrap the OPD signal
    // has to be done before filtering, since filtering smoothes out the jumps
    for (int i = 0; i < 10; i++) {
      while (opd_rad[i] - opd_rad_i_prev > PI) {
        opd_rad[i] -= 2 * PI;
      }
      while (opd_rad[i] - opd_rad_i_prev < -PI) {
        opd_rad[i] += 2 * PI;
      }
      opd_rad_i_prev = opd_rad[i];
    }

    // filter signals
    for (int i = 0; i < 10; i++) {
      opd_rad_f = opd_lp_filter.filter(opd_rad[i]);

      // coordinate system of quad cell is rotated by 45 degrees, hence the combination of basis vectors
      shear_x1_f = shear_x1_lpfilt.filter(shear_y1_um[i] + shear_x1_um[i]);
      shear_x2_f = shear_x2_lpfilt.filter(shear_y2_um[i] + shear_x2_um[i]);
      shear_y1_f = shear_y1_lpfilt.filter(shear_x1_um[i] - shear_y1_um[i]);
      shear_y2_f = shear_y2_lpfilt.filter(shear_x2_um[i] - shear_y2_um[i]);

      point_x1_f = point_x1_lpfilt.filter(point_y1_um[i] + point_x1_um[i]);
      point_x2_f = point_x2_lpfilt.filter(point_y2_um[i] + point_x2_um[i]);
      point_y1_f = point_y1_lpfilt.filter(point_x1_um[i] - point_y1_um[i]);
      point_y2_f = point_y2_lpfilt.filter(point_x2_um[i] - point_y2_um[i]);

      // TODO derive null intensity from science signals
      sci_null = null_lockin.process(adc_sci_null[i], adc_sci_mod[i]);
    }

    // Clamp the OPD signal (to prevent very high values when laser is off)
    if (opd_rad_f > 1e4) {
      opd_rad_f = 9e3;
    }
    if (opd_rad_f < -1e4) {
      opd_rad_f = -9e3;
    }

    // convert OPD from radians to nm
    opd_nm_f = opd_rad_f * 1550 / (2 * PI);

    // enqueue sensor data
    sensorDataQueue.push(
        {t, opd_nm_f, shear_x1_f, shear_x2_f, shear_y1_f, shear_y2_f, point_x1_f, point_x2_f, point_y1_f, point_y2_f});

    sci_null_queue.push({t, sci_null});

    // enqueue adc measurements
    for (int i = 0; i < 10; i++) {
      adc_queues[0].push({counter[i], adc_shear1[i]});
      adc_queues[1].push({counter[i], adc_shear2[i]});
      adc_queues[2].push({counter[i], adc_shear3[i]});
      adc_queues[3].push({counter[i], adc_shear4[i]});
      adc_queues[4].push({counter[i], adc_point1[i]});
      adc_queues[5].push({counter[i], adc_point2[i]});
      adc_queues[6].push({counter[i], adc_point3[i]});
      adc_queues[7].push({counter[i], adc_point4[i]});
      adc_queues[8].push({counter[i], adc_sine_ref[i]});
      adc_queues[9].push({counter[i], adc_opd_ref[i]});
      shear_sum_queue.push({counter[i], adc_shear1[i] + adc_shear2[i] + adc_shear3[i] + adc_shear4[i]});
      point_sum_queue.push({counter[i], adc_point1[i] + adc_point2[i] + adc_point3[i] + adc_point4[i]});
      adc_sci_null_queue.push({counter[i], adc_sci_null[i]});
      adc_sci_mod_queue.push({counter[i], adc_sci_mod[i]});
    }

    // Run control loops
    opd_loop.control(t, opd_nm_f);
    shear_x1_loop.control(t, shear_x1_f);
    shear_x2_loop.control(t, shear_x2_f);
    shear_y1_loop.control(t, shear_y1_f);
    shear_y2_loop.control(t, shear_y2_f);
    point_1_loop.control(t, {point_x1_f, point_y1_f});
    point_2_loop.control(t, {point_x2_f, point_y2_f});

    // wait until t + 7.813 us
    // std::this_thread::sleep_until(t_chrono + std::chrono::nanoseconds(7813));
  }
}

template <class C, class A>
void characterise_open_loop(SISOControlLoop<C, A> &loop, float P, float I, float t_settle, float t_record,
                            std::string filename) {
  // prepare storage file
  std::ofstream file(filename);
  file << "Time (s),OPD (nm),Shear x1 (um),Shear x2 (um),Shear y1 (um),Shear y2 (um),Pointing x1 (urad),Pointing x2 "
          "(urad),Pointing y1 (urad),Pointing y2 (urad)\n";

  loop.reset_all();
  loop.control_mode.store(0);
  loop.p.store(P);
  loop.i.store(I);
  std::this_thread::sleep_for(std::chrono::seconds(int(t_settle)));  // wait settling time
  while (!sensorDataQueue.isempty()) {
    sensorDataQueue.pop();
  }  // flush measurement queue

  // record data for t_record seconds
  auto t_start = std::chrono::high_resolution_clock::now();
  while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - t_start).count() <
         t_record) {
    // wait 10 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // write to file
    if (!sensorDataQueue.isempty()) {
      int N = sensorDataQueue.size();
      for (int i = 0; i < N; i++) {
        auto m = sensorDataQueue.pop();
        // format: 6 decimals for time, 3 decimals for measurement
        file << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3) << m.opd
             << "," << std::fixed << std::setprecision(3) << m.shear_x1 << "," << std::fixed << std::setprecision(3)
             << m.shear_x2 << "," << std::fixed << std::setprecision(3) << m.shear_y1 << "," << std::fixed
             << std::setprecision(3) << m.shear_y2 << "," << std::fixed << std::setprecision(3) << m.point_x1 << ","
             << std::fixed << std::setprecision(3) << m.point_x2 << "," << std::fixed << std::setprecision(3)
             << m.point_y1 << "," << std::fixed << std::setprecision(3) << m.point_y2 << "\n";
      }
    }
  }
  // reset
  loop.reset_all();
}

template <class C, class A>
void characterise_control_loop(SISOControlLoop<C, A> &loop, float P, float I, float t_settle, float t_record, float f1,
                               float f2, float fsteps, float dither_amp, std::string description) {
  std::cout << "Starting control loop characterisation" << std::endl;

  // dither frequencies
  std::vector<float> dither_freqs = {0.1, 0.2, 0.4, 0.8};  // low frequencies take long, so only a few

  // append logarithmic freqs
  float logf1 = std::log10(f1);
  float logf2 = std::log10(f2);
  for (int i = 0; i < fsteps; i++) {
    dither_freqs.push_back(std::pow(10, logf1 + i * (logf2 - logf1) / (fsteps - 1)));
  }

  // dither amplitudes
  std::vector<float> dither_amps(dither_freqs.size(), dither_amp);  // nm

  // recording time: at least 1 s, at most 10/freq
  std::vector<float> recording_times(dither_freqs.size(), 0.0);  // s

  for (int i = 0; i < dither_freqs.size(); i++) {
    recording_times[i] = std::max(1.0, 10.0 / dither_freqs[i]);
  }

  // directory for storage is labelled with ISO datestring + description
  auto datetime_string = get_iso_datestring();
  std::string dirname = "measurements/" + datetime_string + "_" + description;
  std::filesystem::create_directory(dirname);

  // write into parameters.txt: P, I, time, description
  std::string param_filename = dirname + "/parameters.txt";
  std::ofstream param_file(param_filename);
  param_file << "P: " << P << "\n";
  param_file << "I: " << I << "\n";
  param_file << "t_settle: " << t_settle << "\n";
  param_file << "t_record: " << t_record << "\n";
  param_file << "f1: " << f1 << "\n";
  param_file << "f2: " << f2 << "\n";
  param_file << "fsteps: " << fsteps << "\n";
  param_file << "dither_amp: " << dither_amp << "\n";
  param_file << "description: " << description << "\n";
  param_file.close();

  // write into freqs.csv: dither frequencies, dither amplitudes, recording times
  // dither frequency is a string that matches the filename
  std::string freq_filename = dirname + "/freqs.csv";
  std::ofstream freq_file(freq_filename);
  freq_file << "Frequency (Hz),Dither amplitude,Recording time (s)\n";
  for (int i = 0; i < dither_freqs.size(); i++) {
    freq_file << std::to_string(dither_freqs[i]) << "," << dither_amps[i] << "," << recording_times[i] << "\n";
  }
  freq_file.close();

  // reset all
  loop.reset_all();
  loop.p.store(P);
  loop.i.store(I);

  // OPEN LOOP CHARACTERISATION
  std::cout << "\t Open loop time series" << std::endl;
  // storage file
  std::string filename = dirname + "/open_loop.csv";

  characterise_open_loop(loop, P, I, t_settle, t_record, filename);

  // CLOSED LOOP CHARACTERISATION
  std::cout << "\t Closed loop time series" << std::endl;
  // storage file
  filename = dirname + "/closed_loop.csv";
  std::ofstream file(filename);
  file << "Time (s),OPD (nm),Shear x1 (um),Shear x2 (um),Shear y1 (um),Shear y2 (um),Pointing x1 (urad),Pointing x2 "
          "(urad),Pointing y1 (urad),Pointing y2 (urad)\n";

  // reset
  loop.reset_all();
  loop.p.store(P);
  loop.i.store(I);

  // turn on control loop
  loop.control_mode.store(4);

  // wait settling time
  std::this_thread::sleep_for(std::chrono::seconds(int(t_settle)));

  // flush OPD queue
  while (!sensorDataQueue.isempty()) {
    sensorDataQueue.pop();
  }

  // record for recording time: every 10 ms, write contents to file
  auto t_start = std::chrono::high_resolution_clock::now();
  while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - t_start).count() <
         t_record) {
    // wait 10 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // write to file
    if (!sensorDataQueue.isempty()) {
      int N = sensorDataQueue.size();
      for (int i = 0; i < N; i++) {
        auto m = sensorDataQueue.pop();
        // format: 6 decimals for time, 3 decimals for OPD
        file << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3) << m.opd
             << "," << std::fixed << std::setprecision(3) << m.shear_x1 << "," << std::fixed << std::setprecision(3)
             << m.shear_x2 << "," << std::fixed << std::setprecision(3) << m.shear_y1 << "," << std::fixed
             << std::setprecision(3) << m.shear_y2 << "," << std::fixed << std::setprecision(3) << m.point_x1 << ","
             << std::fixed << std::setprecision(3) << m.point_x2 << "," << std::fixed << std::setprecision(3)
             << m.point_y1 << "," << std::fixed << std::setprecision(3) << m.point_y2 << "\n";
      }
    }
  }
  file.close();

  // STEP RESPONSE CHARACTERISATION
  std::cout << "\t Step response" << std::endl;
  // storage file
  filename = dirname + "/step_response.csv";
  file.open(filename);
  file << "Time (s),Measurement,Setpoint,Dither signal,Controller input,Controller output,Actuator command\n";

  // file 2
  std::string filename3 = dirname + "/step_response_sensor.csv";
  std::ofstream file3(filename3);
  file3 << "Time (s),OPD (nm),Shear x1 (um),Shear x2 (um),Shear y1 (um),Shear y2 (um),Pointing x1 (urad),Pointing x2 "
           "(urad),Pointing y1 (urad),Pointing y2 (urad)\n";

  // reset control loop
  loop.reset_all();
  loop.p.store(P);
  loop.i.store(I);

  // settle control loop
  loop.control_mode.store(4);
  std::this_thread::sleep_for(std::chrono::seconds(int(t_settle)));

  // flush data queues
  while (!loop.controlDataBuffer.isempty()) {
    loop.controlDataBuffer.pop();
  }
  while (!sensorDataQueue.isempty()) {
    sensorDataQueue.pop();
  }

  // record for 10 s: every 100 ms, toggle setpoint and record data
  t_start = std::chrono::high_resolution_clock::now();
  bool setpoint_high = false;
  while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - t_start).count() <
         10.0) {
    // wait 100 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // toggle setpoint
    if (setpoint_high) {
      loop.setpoint.store(0.0);
    } else {
      loop.setpoint.store(dither_amp);
    }
    setpoint_high = !setpoint_high;

    // write to file
    if (!loop.controlDataBuffer.isempty()) {
      int N = loop.controlDataBuffer.size();
      for (int i = 0; i < N; i++) {
        auto m = loop.controlDataBuffer.pop();
        // format: 6 decimals for time, 3 decimals for OPD
        file << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3)
             << m.measurement << "," << std::fixed << std::setprecision(3) << m.setpoint << "," << std::fixed
             << std::setprecision(3) << m.dither_signal << "," << std::fixed << std::setprecision(3)
             << m.controller_input << "," << std::fixed << std::setprecision(3) << m.controller_output << ","
             << std::fixed << std::setprecision(3) << m.actuator_command << "\n";
      }
    }

    // write to file 2
    if (!sensorDataQueue.isempty()) {
      int N = sensorDataQueue.size();
      for (int i = 0; i < N; i++) {
        auto m = sensorDataQueue.pop();
        // format: 6 decimals for time, 3 decimals for OPD
        file3 << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3) << m.opd
              << "," << std::fixed << std::setprecision(3) << m.shear_x1 << "," << std::fixed << std::setprecision(3)
              << m.shear_x2 << "," << std::fixed << std::setprecision(3) << m.shear_y1 << "," << std::fixed
              << std::setprecision(3) << m.shear_y2 << "," << std::fixed << std::setprecision(3) << m.point_x1 << ","
              << std::fixed << std::setprecision(3) << m.point_x2 << "," << std::fixed << std::setprecision(3)
              << m.point_y1 << "," << std::fixed << std::setprecision(3) << m.point_y2 << "\n";
      }
    }
  }
  file.close();

  // CONTROL LAW FREQUENCY CHARACTERISATION
  std::cout << "\t Controller frequency characterisation" << std::endl;

  // make subdir: opd_controller_freq
  std::string subdir = dirname + "/freq_controller";
  std::filesystem::create_directory(subdir);

  // seperate storage file for each frequency
  for (int i = 0; i < dither_freqs.size(); i++) {
    std::cout << "\t f = " << dither_freqs[i] << " Hz" << std::endl;
    // storage file: frequency in format 1.2345e67
    filename = subdir + "/" + std::to_string(dither_freqs[i]) + "_hz.csv";
    file.open(filename);
    file << "Time (s),Measurement,Setpoint,Dither signal,Controller input,Controller output,Actuator command\n";

    // reset control loop
    loop.reset_all();
    loop.p.store(P);
    loop.i.store(I);

    // set dither loop parameters
    loop.dither_freq.store(dither_freqs[i]);
    loop.dither_amp.store(dither_amps[i]);

    // settle control loop
    loop.control_mode.store(2);
    std::this_thread::sleep_for(std::chrono::seconds(int(t_settle)));

    // flush data queues
    while (!loop.controlDataBuffer.isempty()) {
      loop.controlDataBuffer.pop();
    }

    // record for recording time: every 10 ms, flush opd queue and write contents to file
    t_start = std::chrono::high_resolution_clock::now();
    while (
        std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - t_start).count() <
        recording_times[i]) {
      // wait 10 ms
      std::this_thread::sleep_for(std::chrono::milliseconds(10));

      // write to file
      if (!loop.controlDataBuffer.isempty()) {
        int N = loop.controlDataBuffer.size();
        for (int i = 0; i < N; i++) {
          auto m = loop.controlDataBuffer.pop();
          // format: 6 decimals for time, 3 decimals for OPD
          file << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3)
               << m.measurement << "," << std::fixed << std::setprecision(3) << m.setpoint << "," << std::fixed
               << std::setprecision(3) << m.dither_signal << "," << std::fixed << std::setprecision(3)
               << m.controller_input << "," << std::fixed << std::setprecision(3) << m.controller_output << ","
               << std::fixed << std::setprecision(3) << m.actuator_command << "\n";
        }
      }
    }

    file.close();
  }

  // FREQUENCY CHARACTERISATION PLANT
  std::cout << "\t Plant frequency characterisation" << std::endl;

  // make subdir: opd_plant_freq
  std::string subdir1 = dirname + "/freq_plant";
  std::filesystem::create_directory(subdir1);

  // seperate storage file for each frequency

  for (int i = 0; i < dither_freqs.size(); i++) {
    std::cout << "\t f = " << dither_freqs[i] << " Hz" << std::endl;
    // storage file: frequency in format 1.2345e67
    filename = subdir1 + "/control_" + std::to_string(dither_freqs[i]) + "_hz.csv";
    file.open(filename);
    file << "Time (s),Measurement,Setpoint,Dither signal,Controller input,Controller output,Actuator command\n";

    // second file: all sensor data
    std::string filename2 = subdir1 + "/sensor_" + std::to_string(dither_freqs[i]) + "_hz.csv";
    std::ofstream file2(filename2);
    file2 << "Time (s),OPD (nm),Shear x1 (um),Shear x2 (um),Shear y1 (um),Shear y2 (um),Pointing x1 (urad),Pointing x2 "
             "(urad),Pointing y1 (urad),Pointing y2 (urad)\n";

    // reset control loop
    loop.reset_all();
    loop.p.store(P);
    loop.i.store(I);

    // set dither loop parameters
    loop.dither_freq.store(dither_freqs[i]);
    loop.dither_amp.store(dither_amps[i]);

    // settle control loop
    loop.control_mode.store(1);
    std::this_thread::sleep_for(std::chrono::seconds(int(t_settle)));

    // flush data queues
    while (!loop.controlDataBuffer.isempty()) {
      loop.controlDataBuffer.pop();
    }

    while (!sensorDataQueue.isempty()) {
      sensorDataQueue.pop();
    }

    // record for recording time: every 100 ms, flush opd queue and write contents to file.
    t_start = std::chrono::high_resolution_clock::now();
    while (
        std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - t_start).count() <
        recording_times[i]) {
      // wait 100 ms
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      // write to file 1
      if (!loop.controlDataBuffer.isempty()) {
        int N = loop.controlDataBuffer.size();
        for (int i = 0; i < N; i++) {
          auto m = loop.controlDataBuffer.pop();
          // format: 6 decimals for time, 3 decimals for OPD
          file << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3)
               << m.measurement << "," << std::fixed << std::setprecision(3) << m.setpoint << "," << std::fixed
               << std::setprecision(3) << m.dither_signal << "," << std::fixed << std::setprecision(3)
               << m.controller_input << "," << std::fixed << std::setprecision(3) << m.controller_output << ","
               << std::fixed << std::setprecision(3) << m.actuator_command << "\n";
        }
      }

      // write to file 2
      if (!sensorDataQueue.isempty()) {
        int N = sensorDataQueue.size();
        for (int i = 0; i < N; i++) {
          auto m = sensorDataQueue.pop();
          // format: 6 decimals for time, 3 decimals for OPD
          file2 << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3) << m.opd
                << "," << std::fixed << std::setprecision(3) << m.shear_x1 << "," << std::fixed << std::setprecision(3)
                << m.shear_x2 << "," << std::fixed << std::setprecision(3) << m.shear_y1 << "," << std::fixed
                << std::setprecision(3) << m.shear_y2 << "," << std::fixed << std::setprecision(3) << m.point_x1 << ","
                << std::fixed << std::setprecision(3) << m.point_x2 << "," << std::fixed << std::setprecision(3)
                << m.point_y1 << "," << std::fixed << std::setprecision(3) << m.point_y2 << "\n";
        }
      }
    }
    file.close();
    file2.close();
  }

  // FREQUENCY CHARACTERISATION CLOSED LOOP DITHER PLANT
  // for all dither frequencies and amplitudes, run the control loop for a while and record dither and opd measurments
  std::cout << "\t Closed loop dither plant" << std::endl;

  std::string subdir2 = dirname + "/freq_closed_loop_dither_plant";
  std::filesystem::create_directory(subdir2);

  for (int i = 0; i < dither_freqs.size(); i++) {
    std::cout << "\t f = " << dither_freqs[i] << " Hz" << std::endl;
    // storage file: frequency in format 1.2345e67
    filename = subdir2 + "/control_" + std::to_string(dither_freqs[i]) + "_hz.csv";
    file.open(filename);
    file << "Time (s),Measurement,Setpoint,Dither signal,Controller input,Controller output,Actuator command\n";

    // file2
    std::string filename2 = subdir2 + "/sensor_" + std::to_string(dither_freqs[i]) + "_hz.csv";
    std::ofstream file2(filename2);
    file2 << "Time (s),OPD (nm),Shear x1 (um),Shear x2 (um),Shear y1 (um),Shear y2 (um),Pointing x1 (urad),Pointing x2 "
             "(urad),Pointing y1 (urad),Pointing y2 (urad)\n";

    // reset control loop
    loop.reset_all();
    loop.p.store(P);
    loop.i.store(I);

    // set dither loop parameters
    loop.dither_freq.store(dither_freqs[i]);
    loop.dither_amp.store(dither_amps[i]);

    // settle control loop
    loop.control_mode.store(4);
    std::this_thread::sleep_for(std::chrono::seconds(int(t_settle)));

    // flush data queues
    while (!loop.controlDataBuffer.isempty()) {
      loop.controlDataBuffer.pop();
    }

    while (!sensorDataQueue.isempty()) {
      sensorDataQueue.pop();
    }

    // record for recording time: every 100 ms, flush opd queue and write contents to file.
    t_start = std::chrono::high_resolution_clock::now();
    while (
        std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - t_start).count() <
        recording_times[i]) {
      // wait 100 ms
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      // write to file
      if (!loop.controlDataBuffer.isempty()) {
        int N = loop.controlDataBuffer.size();
        for (int i = 0; i < N; i++) {
          auto m = loop.controlDataBuffer.pop();
          // format: 6 decimals for time, 3 decimals for OPD
          file << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3)
               << m.measurement << "," << std::fixed << std::setprecision(3) << m.setpoint << "," << std::fixed
               << std::setprecision(3) << m.dither_signal << "," << std::fixed << std::setprecision(3)
               << m.controller_input << "," << std::fixed << std::setprecision(3) << m.controller_output << ","
               << std::fixed << std::setprecision(3) << m.actuator_command << "\n";
        }
      }

      // write to file 2
      if (!sensorDataQueue.isempty()) {
        int N = sensorDataQueue.size();
        for (int i = 0; i < N; i++) {
          auto m = sensorDataQueue.pop();
          // format: 6 decimals for time, 3 decimals for OPD
          file2 << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3) << m.opd
                << "," << std::fixed << std::setprecision(3) << m.shear_x1 << "," << std::fixed << std::setprecision(3)
                << m.shear_x2 << "," << std::fixed << std::setprecision(3) << m.shear_y1 << "," << std::fixed
                << std::setprecision(3) << m.shear_y2 << "," << std::fixed << std::setprecision(3) << m.point_x1 << ","
                << std::fixed << std::setprecision(3) << m.point_x2 << "," << std::fixed << std::setprecision(3)
                << m.point_y1 << "," << std::fixed << std::setprecision(3) << m.point_y2 << "\n";
        }
      }
    }
    file.close();
    file2.close();
  }

  // FREQUENCY CHARACTERISATION CLOSED LOOP DITHER SETPOINT
  // for all dither frequencies and amplitudes, run the control loop for a while and record dither and opd measurments
  std::cout << "\t OPD frequency closed loop dither setpoint" << std::endl;

  std::string subdir3 = dirname + "/freq_closed_loop_dither_setpoint";
  std::filesystem::create_directory(subdir3);

  for (int i = 0; i < dither_freqs.size(); i++) {
    std::cout << "\t f = " << dither_freqs[i] << " Hz" << std::endl;
    // storage file: frequency in format 1.2345e67
    filename = subdir3 + "/control_" + std::to_string(dither_freqs[i]) + "_hz.csv";
    file.open(filename);
    file << "Time (s),Measurement,Setpoint,Dither signal,Controller input,Controller output,Actuator command\n";

    // file2
    std::string filename2 = subdir3 + "/sensor_" + std::to_string(dither_freqs[i]) + "_hz.csv";
    std::ofstream file2(filename2);
    file2 << "Time (s),OPD (nm),Shear x1 (um),Shear x2 (um),Shear y1 (um),Shear y2 (um),Pointing x1 (urad),Pointing x2 "
             "(urad),Pointing y1 (urad),Pointing y2 (urad)\n";

    // reset control loop
    loop.reset_all();
    loop.p.store(P);
    loop.i.store(I);

    // set dither loop parameters
    loop.dither_freq.store(dither_freqs[i]);
    loop.dither_amp.store(dither_amps[i]);

    // settle control loop
    loop.control_mode.store(5);
    std::this_thread::sleep_for(std::chrono::seconds(int(t_settle)));

    // flush data queues
    while (!loop.controlDataBuffer.isempty()) {
      loop.controlDataBuffer.pop();
    }

    while (!sensorDataQueue.isempty()) {
      sensorDataQueue.pop();
    }

    // record for recording time: every 100 ms, flush opd queue and write contents to file.
    t_start = std::chrono::high_resolution_clock::now();
    while (
        std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - t_start).count() <
        recording_times[i]) {
      // wait 100 ms
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      // write to file
      if (!loop.controlDataBuffer.isempty()) {
        int N = loop.controlDataBuffer.size();
        for (int i = 0; i < N; i++) {
          auto m = loop.controlDataBuffer.pop();
          // format: 6 decimals for time, 3 decimals for OPD
          file << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3)
               << m.measurement << "," << std::fixed << std::setprecision(3) << m.setpoint << "," << std::fixed
               << std::setprecision(3) << m.dither_signal << "," << std::fixed << std::setprecision(3)
               << m.controller_input << "," << std::fixed << std::setprecision(3) << m.controller_output << ","
               << std::fixed << std::setprecision(3) << m.actuator_command << "\n";
        }
      }

      // write to file 2
      if (!sensorDataQueue.isempty()) {
        int N = sensorDataQueue.size();
        for (int i = 0; i < N; i++) {
          auto m = sensorDataQueue.pop();
          // format: 6 decimals for time, 3 decimals for OPD
          file2 << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3) << m.opd
                << "," << std::fixed << std::setprecision(3) << m.shear_x1 << "," << std::fixed << std::setprecision(3)
                << m.shear_x2 << "," << std::fixed << std::setprecision(3) << m.shear_y1 << "," << std::fixed
                << std::setprecision(3) << m.shear_y2 << "," << std::fixed << std::setprecision(3) << m.point_x1 << ","
                << std::fixed << std::setprecision(3) << m.point_x2 << "," << std::fixed << std::setprecision(3)
                << m.point_y1 << "," << std::fixed << std::setprecision(3) << m.point_y2 << "\n";
        }
      }
    }
    file.close();
    file2.close();
  }

  // DONE

  // reset control loop
  loop.reset_all();
  loop.p.store(P);
  loop.i.store(I);

  gui_control.store(true);
  std::cout << "Finished control loop characterisation: " << description << std::endl;
}

template <class C1, class A1, class C2, class A2, class A3>
void characterise_joint_closed_loop(SISOControlLoop<C1, A1> &opd_loop, SISOControlLoop<C2, A2> &shear_x1_loop,
                                    SISOControlLoop<C2, A2> &shear_x2_loop, SISOControlLoop<C2, A3> &shear_y1_loop,
                                    SISOControlLoop<C2, A3> &shear_y2_loop, float opd_p, float opd_i, float shear_p,
                                    float shear_i, float t_settle, float t_record, std::string description) {
  std::cout << "Starting joint closed loop characterisation" << std::endl;

  // reset control loops
  opd_loop.reset_all();
  shear_x1_loop.reset_all();
  shear_x2_loop.reset_all();
  shear_y1_loop.reset_all();
  shear_y2_loop.reset_all();

  // set control parameters
  opd_loop.p.store(opd_p);
  opd_loop.i.store(opd_i);

  shear_x1_loop.p.store(shear_p);
  shear_x1_loop.i.store(shear_i);

  shear_x2_loop.p.store(shear_p);
  shear_x2_loop.i.store(shear_i);

  shear_y1_loop.p.store(shear_p);
  shear_y1_loop.i.store(shear_i);

  shear_y2_loop.p.store(shear_p);
  shear_y2_loop.i.store(shear_i);

  // directory to store files in: dire name is opd_date_time (e.g. measurements/opd_2021-09-01_12:00:00)
  auto datetime_string = get_iso_datestring();
  std::string dirname = "measurements/" + datetime_string + "_" + description;
  std::filesystem::create_directory(dirname);

  // CLOSED LOOP CHARACTERISATION
  std::cout << "\t Closed loop time series" << std::endl;
  // storage file
  std::string filename = dirname + "/closed_loop.csv";
  std::ofstream file(filename);

  file << "Time (s),OPD (nm),Shear x1 (um),Shear x2 (um),Shear y1 (um),Shear y2 (um),Pointing x1 (urad),Pointing x2 "
          "(urad),Pointing y1 (urad),Pointing y2 (urad)\n";

  // reset controller
  opd_loop.controller.reset_state();
  shear_x1_loop.controller.reset_state();
  shear_x2_loop.controller.reset_state();
  shear_y1_loop.controller.reset_state();
  shear_y2_loop.controller.reset_state();

  // turn on control loops
  opd_loop.control_mode.store(4);
  shear_x1_loop.control_mode.store(4);
  shear_x2_loop.control_mode.store(4);
  shear_y1_loop.control_mode.store(4);
  shear_y2_loop.control_mode.store(4);

  // wait settling time
  std::this_thread::sleep_for(std::chrono::seconds(int(t_settle)));

  // flush data queues
  while (!sensorDataQueue.isempty()) {
    sensorDataQueue.pop();
  }

  // record for recording time: every 10 ms, write contents to file
  auto t_start = std::chrono::high_resolution_clock::now();
  while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - t_start).count() <
         t_record) {
    // wait 10 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // write to file
    if (!sensorDataQueue.isempty()) {
      int N = sensorDataQueue.size();
      for (int i = 0; i < N; i++) {
        auto m = sensorDataQueue.pop();
        // format: 6 decimals for time, 3 decimals for OPD
        file << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3) << m.opd
             << "," << std::fixed << std::setprecision(3) << m.shear_x1 << "," << std::fixed << std::setprecision(3)
             << m.shear_x2 << "," << std::fixed << std::setprecision(3) << m.shear_y1 << "," << std::fixed
             << std::setprecision(3) << m.shear_y2 << "," << std::fixed << std::setprecision(3) << m.point_x1 << ","
             << std::fixed << std::setprecision(3) << m.point_x2 << "," << std::fixed << std::setprecision(3)
             << m.point_y1 << "," << std::fixed << std::setprecision(3) << m.point_y2 << "\n";
      }
    }
  }

  file.close();

  //  turn off all control loops
  opd_loop.control_mode.store(0);
  shear_x1_loop.control_mode.store(0);
  shear_x2_loop.control_mode.store(0);
  shear_y1_loop.control_mode.store(0);
  shear_y2_loop.control_mode.store(0);

  // DONE
  std::cout << "Finished joint closed loop characterisation: " << description << std::endl;
}

void startMeasurement() {
  RunMeasurement.store(true);
  RunMeasurement.notify_all();
}

void stopMeasurement() { RunMeasurement.store(false); }

void RenderUI() {
  ImGui::Begin("NICE Control");

  ImGuiIO &io = ImGui::GetIO();

  // OPD control <-> GUI
  static int gui_opd_loop_select = 0;
  static float opd_p_gui = 0.700f;
  static float opd_i_gui = 0.009f;
  static float opd_dither_freq_gui = 0.0f;
  static float opd_dither_amp_gui = 0.0f;

  // shear control <-> GUI
  static int shear_loop_select = 0;
  static float shear_p_gui = 0.4f;
  static float shear_i_gui = 0.007f;

  // pointing control <-> GUI
  static int pointing_loop_select = 0;
  static float pointing_p_gui = 1e-5f;
  static float pointing_i_gui = 1e-7f;

  if (gui_control.load()) {
    // OPD
    if (gui_opd_loop_select == 2) {
      opd_loop.control_mode.store(4);
    } else if (gui_opd_loop_select == 1) {
      opd_loop.control_mode.store(1);
    } else {
      opd_loop.control_mode.store(0);
    }

    opd_loop.setpoint.store(opd_setpoint_gui);
    opd_loop.p.store(opd_p_gui);
    opd_loop.i.store(opd_i_gui);
    opd_loop.dither_freq.store(opd_dither_freq_gui);
    opd_loop.dither_amp.store(opd_dither_amp_gui);

    // Shear
    if (shear_loop_select == 2) {
      shear_x1_loop.control_mode.store(4);
      shear_x2_loop.control_mode.store(4);
      shear_y1_loop.control_mode.store(4);
      shear_y2_loop.control_mode.store(4);
    } else if (shear_loop_select == 1) {
      shear_x1_loop.control_mode.store(1);
      shear_x2_loop.control_mode.store(1);
      shear_y1_loop.control_mode.store(1);
      shear_y2_loop.control_mode.store(1);
    } else {
      shear_x1_loop.control_mode.store(0);
      shear_x2_loop.control_mode.store(0);
      shear_y1_loop.control_mode.store(0);
      shear_y2_loop.control_mode.store(0);
    }

    shear_x1_loop.p.store(shear_p_gui);
    shear_x1_loop.i.store(shear_i_gui);
    shear_x2_loop.p.store(shear_p_gui);
    shear_x2_loop.i.store(shear_i_gui);
    shear_y1_loop.p.store(shear_p_gui);
    shear_y1_loop.i.store(shear_i_gui);
    shear_y2_loop.p.store(shear_p_gui);
    shear_y2_loop.i.store(shear_i_gui);

    shear_x1_loop.setpoint.store(shear_x1_setpoint_gui);
    shear_x2_loop.setpoint.store(shear_x2_setpoint_gui);
    shear_y1_loop.setpoint.store(shear_y1_setpoint_gui);
    shear_y2_loop.setpoint.store(shear_y2_setpoint_gui);

    // Pointing
    if (pointing_loop_select == 2) {
      point_1_loop.plant_mode.store({2, 2});
      point_1_loop.controller_mode.store({1, 1});
      point_2_loop.plant_mode.store({2, 2});
      point_2_loop.controller_mode.store({1, 1});
    } else if (pointing_loop_select == 1) {
      point_1_loop.plant_mode.store({1, 1});
      point_1_loop.controller_mode.store({0, 0});
      point_2_loop.plant_mode.store({1, 1});
      point_2_loop.controller_mode.store({0, 0});
    } else {
      point_1_loop.plant_mode.store({0, 0});
      point_1_loop.controller_mode.store({0, 0});
      point_2_loop.plant_mode.store({0, 0});
      point_2_loop.controller_mode.store({0, 0});
    }

    point_1_loop.Ps.store({pointing_p_gui, pointing_p_gui});
    point_1_loop.Is.store({pointing_i_gui, pointing_i_gui});
    point_2_loop.Ps.store({pointing_p_gui, pointing_p_gui});
    point_2_loop.Is.store({pointing_i_gui, pointing_i_gui});

    point_1_loop.setpoint.store({{pointing_x1_setpoint_gui, pointing_y1_setpoint_gui}});
    point_2_loop.setpoint.store({{pointing_x2_setpoint_gui, pointing_y2_setpoint_gui}});
  }

  static ScrollingBuffer opd_buffer, shear_x1_buffer, shear_x2_buffer, shear_y1_buffer, shear_y2_buffer,
      point_x1_buffer, point_x2_buffer, point_y1_buffer, point_y2_buffer;

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
    ImGui::Text("Recording OPD measurements to file");
    if (ImGui::Button("Stop recording")) {
      recording_running = false;
      file.close();
    }
  } else {
    ImGui::Text("Record OPD measurements to file");
    if (ImGui::Button("Start recording")) {
      recording_running = true;
      filename = "measurements/" + get_iso_datestring() + "_opd.csv";
      file.open(filename);
      file << "Time at start of measurement: " << get_iso_datestring() << "\n";
      file << "Time (s),OPD (nm)\n";
    }
  }

  // add the enture sensor data queue to the plot buffers
  if (!sensorDataQueue.isempty()) {
    int N = sensorDataQueue.size();
    for (int i = 0; i < N; i++) {
      auto m = sensorDataQueue.pop();
      opd_buffer.AddPoint(m.time, m.opd);
      shear_x1_buffer.AddPoint(m.time, m.shear_x1);
      shear_x2_buffer.AddPoint(m.time, m.shear_x2);
      shear_y1_buffer.AddPoint(m.time, m.shear_y1);
      shear_y2_buffer.AddPoint(m.time, m.shear_y2);
      point_x1_buffer.AddPoint(m.time, m.point_x1);
      point_x2_buffer.AddPoint(m.time, m.point_x2);
      point_y1_buffer.AddPoint(m.time, m.point_y1);
      point_y2_buffer.AddPoint(m.time, m.point_y2);

      // TODO: if saving data, write to file
      if (recording_running) {
        file << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(2) << m.opd
             << "\n";
      }
    }
  }

  // Start measurement
  static bool measure_button = true;
  ImGui::Checkbox("Run measurement", &measure_button);
  if (measure_button) {
    startMeasurement();
  } else {
    stopMeasurement();
  }

  // ADC measurements
  if (ImGui::CollapsingHeader("ADC Measurements")) {
    static ScrollingBufferT<int, int> adc_buffers[10];
    static ScrollingBufferT<int, int> shear_sum_buffer, point_sum_buffer;
    static ScrollingBufferT<int, int> adc_sci_null_buffer, adc_sci_mod_buffer;
    static float t_adc = 0;

    // get lates time in ADC queue
    if (RunMeasurement.load()) {
      if (!adc_queues[0].isempty()) {
        t_adc = adc_queues[0].back().time;
      }
    }

    // add all measurements to the plot buffers
    for (int i = 0; i < 10; i++) {
      if (!adc_queues[i].isempty()) {
        int N = adc_queues[i].size();
        for (int j = 0; j < N; j++) {
          auto m = adc_queues[i].pop();
          adc_buffers[i].AddPoint(m.time, m.value);
        }
      }
    }

    // shear sum
    if (!shear_sum_queue.isempty()) {
      int N = shear_sum_queue.size();
      for (int j = 0; j < N; j++) {
        auto m = shear_sum_queue.pop();
        shear_sum_buffer.AddPoint(m.time, m.value);
      }
    }

    // pointing
    if (!point_sum_queue.isempty()) {
      int N = point_sum_queue.size();
      for (int j = 0; j < N; j++) {
        auto m = point_sum_queue.pop();
        point_sum_buffer.AddPoint(m.time, m.value);
      }
    }

    // sci null
    if (!adc_sci_null_queue.isempty()) {
      int N = adc_sci_null_queue.size();
      for (int j = 0; j < N; j++) {
        auto m = adc_sci_null_queue.pop();
        adc_sci_null_buffer.AddPoint(m.time, m.value);
      }
    }

    // sci mod
    if (!adc_sci_mod_queue.isempty()) {
      int N = adc_sci_mod_queue.size();
      for (int j = 0; j < N; j++) {
        auto m = adc_sci_mod_queue.pop();
        adc_sci_mod_buffer.AddPoint(m.time, m.value);
      }
    }

    // print peak-to-peak and mean value of OPDRef
    if (!adc_buffers[9].Data.empty()) {
      auto last_1000_measurements_data = adc_buffers[9].GetLastN(1000);
      std::vector<int> last_1000_measurements;
      for (auto &m : last_1000_measurements_data) {
        last_1000_measurements.push_back(m.value);
      }
      int min = *std::min_element(last_1000_measurements.begin(), last_1000_measurements.end());
      int max = *std::max_element(last_1000_measurements.begin(), last_1000_measurements.end());
      float mean = std::accumulate(last_1000_measurements.begin(), last_1000_measurements.end(), 0.0) /
                   last_1000_measurements.size();
      // print peak-to-peak as floating point number, e.g. 9.432E4
      ImGui::Text("Peak-to-peak OPDRef: %.2E", float(max - min));
      ImGui::Text("Mean OPDRef: %.2E", mean);
    }

    // same for shear sum
    if (!adc_buffers[0].Data.empty()) {
      auto last_1000_shear_sum_data = shear_sum_buffer.GetLastN(1000);
      std::vector<int> last_1000_shear_sum;
      for (auto &m : last_1000_shear_sum_data) {
        last_1000_shear_sum.push_back(m.value);
      }
      int min = *std::min_element(last_1000_shear_sum.begin(), last_1000_shear_sum.end());
      int max = *std::max_element(last_1000_shear_sum.begin(), last_1000_shear_sum.end());
      float mean =
          std::accumulate(last_1000_shear_sum.begin(), last_1000_shear_sum.end(), 0.0) / last_1000_shear_sum.size();
      // print peak-to-peak as floating point number, e.g. 9.432E4
      ImGui::Text("Peak-to-peak ShearSum: %.2E", float(max - min));
      ImGui::Text("Mean ShearSum: %.2E", mean);
    }

    // same for pointing sum
    if (!adc_buffers[4].Data.empty()) {
      auto last_1000_point_sum_data = point_sum_buffer.GetLastN(1000);
      std::vector<int> last_1000_point_sum;
      for (auto &m : last_1000_point_sum_data) {
        last_1000_point_sum.push_back(m.value);
      }
      int min = *std::min_element(last_1000_point_sum.begin(), last_1000_point_sum.end());
      int max = *std::max_element(last_1000_point_sum.begin(), last_1000_point_sum.end());
      float mean =
          std::accumulate(last_1000_point_sum.begin(), last_1000_point_sum.end(), 0.0) / last_1000_point_sum.size();
      // print peak-to-peak as floating point number, e.g. 9.432E4
      ImGui::Text("Peak-to-peak PointSum: %.2E", float(max - min));
      ImGui::Text("Mean PointSum: %.2E", mean);
    }

    static float adc_history_length = 1000.f;
    ImGui::SliderFloat("ADC History", &adc_history_length, 1, 128000, "%.5f s", ImGuiSliderFlags_Logarithmic);

    // plot label names
    static const char *plot_labels[10] = {"Shear UP",   "Shear LEFT",  "Shear RIGHT", "Shear DOWN", "Point UP",
                                          "Point LEFT", "Point RIGHT", "Point DOWN",  "SineRef",    "OPDRef"};

    // plot style
    static float thickness = 2;
    static ImPlotAxisFlags xflags = ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoTickLabels;
    static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;

    if (ImPlot::BeginPlot("##ADC", ImVec2(-1, 400 * io.FontGlobalScale))) {
      ImPlot::SetupAxes(nullptr, nullptr, xflags, yflags);
      ImPlot::SetupAxisLimits(ImAxis_X1, t_adc - adc_history_length, t_adc, ImGuiCond_Always);
      ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
      ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
      for (int i = 0; i < 10; i++) {
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(i), thickness);
        ImPlot::PlotLine(plot_labels[i], &adc_buffers[i].Data[0].time, &adc_buffers[i].Data[0].value,
                         adc_buffers[i].Data.size(), 0, adc_buffers[i].Offset, 2 * sizeof(int));
      }

      // shear sum
      ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(10), thickness);
      ImPlot::PlotLine("ShearSum", &shear_sum_buffer.Data[0].time, &shear_sum_buffer.Data[0].value,
                       shear_sum_buffer.Data.size(), 0, shear_sum_buffer.Offset, 2 * sizeof(int));

      // pointing sum
      ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(11), thickness);
      ImPlot::PlotLine("PointSum", &point_sum_buffer.Data[0].time, &point_sum_buffer.Data[0].value,
                       point_sum_buffer.Data.size(), 0, point_sum_buffer.Offset, 2 * sizeof(int));

      // sci null
      ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(12), thickness);
      ImPlot::PlotLine("SCINull", &adc_sci_null_buffer.Data[0].time, &adc_sci_null_buffer.Data[0].value,
                       adc_sci_null_buffer.Data.size(), 0, adc_sci_null_buffer.Offset, 2 * sizeof(int));

      // sci mod
      ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(13), thickness);
      ImPlot::PlotLine("SCIMod", &adc_sci_mod_buffer.Data[0].time, &adc_sci_mod_buffer.Data[0].value,
                       adc_sci_mod_buffer.Data.size(), 0, adc_sci_mod_buffer.Offset, 2 * sizeof(int));

      ImPlot::EndPlot();
    }
  }

  // Sci_null measurements
  if (ImGui::CollapsingHeader("Science beam Measurements")) {
    static ScrollingBuffer sci_null_buffer;
    static float t_sci_null = 0;

    // get lates time in Sci_null queue
    if (RunMeasurement.load()) {
      if (!sci_null_queue.isempty()) {
        t_sci_null = sci_null_queue.back().time;
      }
    }

    // add all measurements to the plot buffers
    if (!sci_null_queue.isempty()) {
      int N = sci_null_queue.size();
      for (int i = 0; i < N; i++) {
        auto m = sci_null_queue.pop();
        sci_null_buffer.AddPoint(m.time, m.value);
      }
    }

    static float sci_null_history_length = 1.0e-3;
    ImGui::SliderFloat("Sci_null History", &sci_null_history_length, 0.001, 5., "%.3f s", ImGuiSliderFlags_Logarithmic);

    // plot style
    static float thickness = 3;
    static ImPlotAxisFlags xflags = ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoTickLabels;
    static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;

    if (ImPlot::BeginPlot("##Sci_null", ImVec2(-1, 600 * io.FontGlobalScale))) {
      ImPlot::SetupAxes(nullptr, nullptr, xflags, yflags);
      ImPlot::SetupAxisLimits(ImAxis_X1, t_sci_null - sci_null_history_length, t_sci_null, ImGuiCond_Always);
      ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
      ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
      ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(0), thickness);
      ImPlot::PlotLine("Sci_null", &sci_null_buffer.Data[0].x, &sci_null_buffer.Data[0].y, sci_null_buffer.Data.size(),
                       0, sci_null_buffer.Offset, 2 * sizeof(int));
      ImPlot::EndPlot();
    }
  }

  if (ImGui::CollapsingHeader("OPD")) {
    // characterise button: launch characterisation of OPD thread and deactivate gui control
    static bool opd_char_button = false;

    ImGui::Checkbox("Characterise Control loop", &opd_char_button);

    // print status of gui_control
    ImGui::Text("GUI control: %s", gui_control.load() ? "true" : "false");

    if (opd_char_button) {
      gui_control.store(false);
      opd_char_button = false;

      std::cout << "Waiting for one minute" << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(60));
      std::cout << "Done waiting" << std::endl;

      // loop, p, i, t_settle, t_record, f1, f2, fstep, dither_amp, description
      // characterise_control_loop(opd_loop, 0.7, 0.01, 1.0, 200.0, 1.0, 1000.0, 150, 50.0, "opd_no_box_overnight");
      characterise_control_loop(shear_x1_loop, 0.4, 0.007, 1.0, 200.0, 1.0, 300.0, 150, 50.0, "shear_x1_no_box_repeat");
      // characterise_joint_closed_loop(opd_loop, shear_x1_loop, shear_x2_loop, shear_y1_loop, shear_y2_loop, 0.7, 0.01,
      //  0.4, 0.007, 1.0, 200.0, "joint_no_box_overnight");
    }

    // control mode selector
    ImGui::Text("Control mode:");
    ImGui::SameLine();
    ImGui::RadioButton("Off##OPD", &gui_opd_loop_select, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Open loop##OPD", &gui_opd_loop_select, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Closed loop##OPD", &gui_opd_loop_select, 2);

    // real time plot
    static ScrollingBuffer opd_dith_buffer, setpoint_buffer;

    static float t_gui = 0;
    t_gui = getTime();

    // get control signals
    if (!opd_loop.controlDataBuffer.isempty()) {
      while (!opd_loop.controlDataBuffer.isempty()) {
        auto m = opd_loop.controlDataBuffer.pop();
        setpoint_buffer.AddPoint(m.time, m.setpoint);
        opd_dith_buffer.AddPoint(m.time, m.dither_signal);
      }
    }

    static bool plot_setpoint = false;
    ImGui::Checkbox("Plot setpoint", &plot_setpoint);

    static float opd_history_length = 10.0f;
    ImGui::SliderFloat("OPD History", &opd_history_length, 0.1, 10, "%.2f s", ImGuiSliderFlags_Logarithmic);

    // x axis: no ticks
    static ImPlotAxisFlags xflags = ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoTickLabels;

    // y axis: auto fit
    static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;

    static ImVec4 fft_color = ImVec4(1, 1, 0, 1);
    static ImVec4 setpoint_color = ImVec4(1, 1, 1, 1);
    static float thickness = 1;

    if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1, 200 * io.FontGlobalScale))) {
      ImPlot::SetupAxes(nullptr, nullptr, xflags, yflags);
      ImPlot::SetupAxisLimits(ImAxis_X1, t_gui - opd_history_length, t_gui, ImGuiCond_Always);
      ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
      ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
      ImPlot::SetNextLineStyle(fft_color, thickness);
      ImPlot::PlotLine("Measurement", &opd_buffer.Data[0].x, &opd_buffer.Data[0].y, opd_buffer.Data.size(), 0,
                       opd_buffer.Offset, 2 * sizeof(float));
      if (plot_setpoint) {
        ImPlot::SetNextLineStyle(setpoint_color, thickness);
        ImPlot::PlotLine("Setpoint", &setpoint_buffer.Data[0].x, &setpoint_buffer.Data[0].y,
                         setpoint_buffer.Data.size(), 0, setpoint_buffer.Offset, 2 * sizeof(float));
      }
      ImPlot::EndPlot();
    }

    // plot for dither signal
    if (ImPlot::BeginPlot("Dither##OPD", ImVec2(-1, 150 * io.FontGlobalScale))) {
      ImPlot::SetupAxes(nullptr, nullptr, xflags, yflags);
      ImPlot::SetupAxisLimits(ImAxis_X1, t_gui - opd_history_length, t_gui, ImGuiCond_Always);
      ImPlot::SetupAxisLimits(ImAxis_Y1, -opd_loop.dither_amp.load(), opd_loop.dither_amp.load());
      ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
      ImPlot::SetNextLineStyle(fft_color, thickness);
      ImPlot::PlotLine("Dither signal", &opd_dith_buffer.Data[0].x, &opd_dith_buffer.Data[0].y,
                       opd_dith_buffer.Data.size(), 0, opd_dith_buffer.Offset, 2 * sizeof(float));
      ImPlot::EndPlot();
    }

    if (ImGui::TreeNode("FFT##OPD")) {
      // set up fft
      const static int fft_size = 1024 * 8 * 8;
      static double fft_power[fft_size / 2];
      static double fft_freq[fft_size / 2];
      static FFT_calculator fft(fft_size, 12800., &opd_buffer, fft_power, fft_freq);

      // calculate fft
      fft.calculate();

      // calculate dither fft
      static double fft_power_dith[fft_size / 2];
      static double fft_freq_dith[fft_size / 2];
      static FFT_calculator dither_fft(fft_size, 12800., &opd_dith_buffer, fft_power_dith, fft_freq_dith);
      dither_fft.calculate();

      // plot fft_power vs fft_freq, with log scale on x and y axis
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
        ImPlot::PlotLine("OPD FFT", &fft_freq_dith[0], &fft_power[0], fft_size / 2);
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(1), fft_thickness);
        ImPlot::PlotLine("Dither FFT", &fft_freq_dith[0], &fft_power_dith[0], fft_size / 2);
        ImPlot::EndPlot();
      }

      // plot the ratio of the two ffts
      static double fft_ratio[fft_size / 2];
      for (int i = 0; i < fft_size / 2; i++) {
        fft_ratio[i] = fft_power_dith[i] / fft_power[i];
      }

      if (ImPlot::BeginPlot("##FFT Ratio", ImVec2(-1, 400 * io.FontGlobalScale))) {
        ImPlot::SetupAxes(nullptr, nullptr, fft_xflags, fft_yflags);
        ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
        ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
        ImPlot::SetupAxisLimits(ImAxis_X1, 0.1, 2000);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0.1, 100);
        ImPlot::SetNextLineStyle(fft_color, thickness);
        ImPlot::PlotLine("FFT Ratio", &fft_freq_dith[0], &fft_ratio[0], fft_size / 2);
        ImPlot::EndPlot();
      }

      ImGui::TreePop();
    }

    if (ImGui::TreeNode("Metrology##OPD")) {
      // calculate mean and std of OPD buffer
      static float mean = 0.0f;
      static float stddev = 0.0f;
      const int N_points_stats = 10000;

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
      ImGui::Text("Std: %.4f", stddev);

      ImGui::TreePop();
    }

    if (ImGui::TreeNode("Actuator##OPD")) {
      // plot for current piezo position
      static ImVec4 color = ImVec4(1, 1, 0, 1);
      static ScrollingBuffer piezo_buffer;
      piezo_buffer.AddPoint(t_gui, opd_stage.read());

      if (ImPlot::BeginPlot("##Piezo", ImVec2(-1, 150 * io.FontGlobalScale))) {
        ImPlot::SetupAxes(nullptr, nullptr, xflags, yflags);
        ImPlot::SetupAxisLimits(ImAxis_X1, t_gui - opd_history_length, t_gui, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
        ImPlot::SetNextLineStyle(color, thickness);
        ImPlot::PlotLine("Piezo position", &piezo_buffer.Data[0].x, &piezo_buffer.Data[0].y, piezo_buffer.Data.size(),
                         0, piezo_buffer.Offset, 2 * sizeof(float));
        ImPlot::EndPlot();
      }

      ImGui::TreePop();
    }

    if (ImGui::TreeNode("Control loop##OPD")) {
      ImGui::Text("Control loop parameters:");

      // sliders for p ,i
      ImGui::SliderFloat("P##OPD", &opd_p_gui, 1e-4f, 1e2f, "%.5f", ImGuiSliderFlags_Logarithmic);
      ImGui::SliderFloat("I##OPD", &opd_i_gui, 1e-4f, 3e-1f, "%.5f", ImGuiSliderFlags_Logarithmic);

      const float opd_setpoint_min = -1e6, opd_setpoint_max = 1e6;

      // opd input: drag or ctrl+click to input
      ImGui::DragFloat("OPD Setpoint", &opd_setpoint_gui, 0.1f, opd_setpoint_min, opd_setpoint_max, "%.2f nm",
                       ImGuiSliderFlags_AlwaysClamp);

      // dither parameters
      ImGui::SliderFloat("Dither frequency##OPD", &opd_dither_freq_gui, 0.1f, 1000.0f, "%.2f Hz",
                         ImGuiSliderFlags_Logarithmic);
      ImGui::SliderFloat("Dither amplitude##OPD", &opd_dither_amp_gui, 0.0f, 100.0f, "%.2f nm");

      ImGui::TreePop();
    }
  }

  if (ImGui::CollapsingHeader("Shear")) {
    // control mode selector
    ImGui::Text("Control mode:");
    ImGui::SameLine();
    ImGui::RadioButton("Off##X", &shear_loop_select, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Open loop##X", &shear_loop_select, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Closed loop##X", &shear_loop_select, 2);

    static float t_gui_x = 0;

    // if measurement is running, update gui time.
    t_gui_x = getTime();

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
      ImPlot::PlotLine("Shear X1", &shear_x1_buffer.Data[0].x, &shear_x1_buffer.Data[0].y, shear_x1_buffer.Data.size(),
                       0, shear_x1_buffer.Offset, 2 * sizeof(float));
      ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(1), x1_thickness);
      ImPlot::PlotLine("Shear Y1", &shear_y1_buffer.Data[0].x, &shear_y1_buffer.Data[0].y, shear_y1_buffer.Data.size(),
                       0, shear_y1_buffer.Offset, 2 * sizeof(float));
      ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(2), x1_thickness);
      ImPlot::PlotLine("Shear X2", &shear_x2_buffer.Data[0].x, &shear_x2_buffer.Data[0].y, shear_x2_buffer.Data.size(),
                       0, shear_x2_buffer.Offset, 2 * sizeof(float));
      ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(3), x1_thickness);
      ImPlot::PlotLine("Shear Y2", &shear_y2_buffer.Data[0].x, &shear_y2_buffer.Data[0].y, shear_y2_buffer.Data.size(),
                       0, shear_y2_buffer.Offset, 2 * sizeof(float));
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
      static ImPlotAxisFlags fft_yflags = ImPlotAxisFlags_None;  // ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
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

    if (ImGui::TreeNode("Actuator##Shear")) {
      // plot for current piezo position
      static ScrollingBufferT<double, double> tip_tilt_actuator_buffer[4];
      tip_tilt_actuator_buffer[0].AddPoint(t_gui_x, tip_tilt_stage1.readx());
      tip_tilt_actuator_buffer[1].AddPoint(t_gui_x, tip_tilt_stage1.ready());
      tip_tilt_actuator_buffer[2].AddPoint(t_gui_x, tip_tilt_stage2.readx());
      tip_tilt_actuator_buffer[3].AddPoint(t_gui_x, tip_tilt_stage2.ready());

      // labels
      static const char *plot_labels[4] = {"Tip/Tilt 1 X", "Tip/Tilt 1 Y", "Tip/Tilt 2 X", "Tip/Tilt 2 Y"};

      if (ImPlot::BeginPlot("##Tip/Tilt actuator", ImVec2(-1, 150 * io.FontGlobalScale))) {
        ImPlot::SetupAxes(nullptr, nullptr, x1_xflags, x1_yflags);
        ImPlot::SetupAxisLimits(ImAxis_X1, t_gui_x - x1_history_length, t_gui_x, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
        for (int i = 0; i < 4; i++) {
          ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(i), x1_thickness);
          ImPlot::PlotLine(plot_labels[i], &tip_tilt_actuator_buffer[i].Data[0].time,
                           &tip_tilt_actuator_buffer[i].Data[0].value, tip_tilt_actuator_buffer[i].Data.size(), 0,
                           tip_tilt_actuator_buffer[i].Offset, 2 * sizeof(double));
        }
        ImPlot::EndPlot();
      }

      float tip_tilt_actuator_measurement = 0.0f;

      // Display measurement
      ImGui::Text("Current measurement: %.4f", tip_tilt_actuator_measurement);

      ImGui::TreePop();
    }

    // control
    if (ImGui::TreeNode("Control loop##X1D")) {
      ImGui::Text("Control loop parameters:");

      // sliders for p ,i
      ImGui::SliderFloat("P##X1D", &shear_p_gui, 0.0f, 1.0f);
      ImGui::SliderFloat("I##X1D", &shear_i_gui, 0.0f, 0.05f);

      const float shear_setpoint_min = -1e6, shear_setpoint_max = 1e6;

      // x1d input: drag
      ImGui::DragFloat("Setpoint X1", &shear_x1_setpoint_gui, 0.1f, shear_setpoint_min, shear_setpoint_max, "%.1f",
                       ImGuiSliderFlags_AlwaysClamp);
      ImGui::DragFloat("Setpoint Y1", &shear_y1_setpoint_gui, 0.1f, shear_setpoint_min, shear_setpoint_max, "%.1f",
                       ImGuiSliderFlags_AlwaysClamp);
      ImGui::DragFloat("Setpoint X2", &shear_x2_setpoint_gui, 0.1f, shear_setpoint_min, shear_setpoint_max, "%.1f",
                       ImGuiSliderFlags_AlwaysClamp);
      ImGui::DragFloat("Setpoint Y2", &shear_y2_setpoint_gui, 0.1f, shear_setpoint_min, shear_setpoint_max, "%.1f",
                       ImGuiSliderFlags_AlwaysClamp);

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
    t_gui_x = getTime();

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

    // actuator
    if (ImGui::TreeNode("Actuator##Pointing")) {
      // plot for current piezo position
      static ImVec4 color = ImVec4(1, 1, 0, 1);
      static ScrollingBufferT<double, double> nF_stage_position_buffer[4];
      std::array<float, 2> meas = nF_stage_1.read();
      nF_stage_position_buffer[0].AddPoint(t_gui_x, meas[0]);
      nF_stage_position_buffer[1].AddPoint(t_gui_x, meas[1]);
      // meas = nF_stage_2.read();
      nF_stage_position_buffer[2].AddPoint(t_gui_x, meas[0]);
      nF_stage_position_buffer[3].AddPoint(t_gui_x, meas[1]);

      if (ImPlot::BeginPlot("##nF stage", ImVec2(-1, 150 * io.FontGlobalScale))) {
        ImPlot::SetupAxes(nullptr, nullptr, x1_xflags, x1_yflags);
        ImPlot::SetupAxisLimits(ImAxis_X1, t_gui_x - pointing_history_length, t_gui_x, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(0), pointing_thickness);
        ImPlot::PlotLine("nF stage 1 x", &nF_stage_position_buffer[0].Data[0].time,
                         &nF_stage_position_buffer[0].Data[0].value, nF_stage_position_buffer[0].Data.size(), 0,
                         nF_stage_position_buffer[0].Offset, 2 * sizeof(int));
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(1), pointing_thickness);
        ImPlot::PlotLine("nF stage 1 y", &nF_stage_position_buffer[1].Data[0].time,
                         &nF_stage_position_buffer[1].Data[0].value, nF_stage_position_buffer[1].Data.size(), 0,
                         nF_stage_position_buffer[1].Offset, 2 * sizeof(int));
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(2), pointing_thickness);
        ImPlot::PlotLine("nF stage 2 x", &nF_stage_position_buffer[2].Data[0].time,
                         &nF_stage_position_buffer[2].Data[0].value, nF_stage_position_buffer[2].Data.size(), 0,
                         nF_stage_position_buffer[2].Offset, 2 * sizeof(int));
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(3), pointing_thickness);
        ImPlot::PlotLine("nF stage 2 y", &nF_stage_position_buffer[3].Data[0].time,
                         &nF_stage_position_buffer[3].Data[0].value, nF_stage_position_buffer[3].Data.size(), 0,
                         nF_stage_position_buffer[3].Offset, 2 * sizeof(int));
        ImPlot::EndPlot();
      }

      ImGui::TreePop();
    }

    // pointing control
    if (ImGui::TreeNode("Control loop##Pointing")) {
      ImGui::Text("Control loop parameters:");

      // sliders for p ,i
      ImGui::SliderFloat("P##Pointing", &pointing_p_gui, 1e-8f, 1e-2f, "%.8f", ImGuiSliderFlags_Logarithmic);
      ImGui::SliderFloat("I##Pointing", &pointing_i_gui, 1e-8f, 1e-5f, "%.8f", ImGuiSliderFlags_Logarithmic);

      const float pointing_setpoint_min = -2.0f, pointing_setpoint_max = 2.0f;

      // pointing input: drag
      ImGui::SliderFloat("Pointing X1 setpoint", &pointing_x1_setpoint_gui, pointing_setpoint_min,
                         pointing_setpoint_max, "%.1f nm", ImGuiSliderFlags_AlwaysClamp);
      ImGui::SliderFloat("Pointing Y1 setpoint", &pointing_y1_setpoint_gui, pointing_setpoint_min,
                         pointing_setpoint_max, "%.1f nm", ImGuiSliderFlags_AlwaysClamp);
      ImGui::SliderFloat("Pointing X2 setpoint", &pointing_x2_setpoint_gui, pointing_setpoint_min,
                         pointing_setpoint_max, "%.1f nm", ImGuiSliderFlags_AlwaysClamp);
      ImGui::SliderFloat("Pointing Y2 setpoint", &pointing_y2_setpoint_gui, pointing_setpoint_min,
                         pointing_setpoint_max, "%.1f nm", ImGuiSliderFlags_AlwaysClamp);

      ImGui::TreePop();
    }
  }

  if (ImGui::CollapsingHeader("Program settings")) {
    ImGui::DragFloat("GUI scale", &io.FontGlobalScale, 0.005f, 0.5, 6.0, "%.2f",
                     ImGuiSliderFlags_AlwaysClamp);  // Scale everything
  }
  ImGui::End();

  // demo window
  // static bool show_app_metrics = true;
  // ImGui::ShowDemoWindow();
  // ImGui::ShowMetricsWindow(&show_app_metrics);

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
  config.OversampleH = 1;
  config.OversampleV = 1;
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

  std::cout << "Starting run_calculation" << std::endl;
  std::thread computeThread(NICEcontrol::run_calculation);

  // call setupActuators
  std::cout << "Starting setupActuators" << std::endl;
  NICEcontrol::setupActuators();

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
  { NICEcontrol::RunMeasurement.store(false); }
  computeThread.join();

  return 0;
}
