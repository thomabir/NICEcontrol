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
#include <boost/circular_buffer.hpp>
#include <chrono>
#include <fstream>
#include <future>
#include <iostream>
#include <mutex>
#include <queue>
#include <random>
#include <thread>
#include <filesystem>

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
  //strftime(buf, sizeof buf, "%Y-%m-%dT%H:%M:%SZ", gmtime(&now));
  return buf;
}

struct Measurement {
  double time;
  float value;
};

struct ControlData {
  double time;
  float setpoint;
  float measurement;
  // float error = setpoint - measurement;
  float control_signal;
  float dither_signal;
  // float dithered_control_signal = control_signal + dither_signal;
};


template <typename T, typename U>
struct MeasurementT {
  T time;
  U value;
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

// iir filters
Iir::Butterworth::LowPass<2> shear_x1_lp_filter, shear_x2_lp_filter;
Iir::Butterworth::LowPass<2> shear_x1_control_lp_filter, shear_x2_control_lp_filter, shear_y1_control_lp_filter, shear_y2_control_lp_filter;
Iir::Butterworth::LowPass<2> pointing_x1_control_lp_filter, pointing_x2_control_lp_filter, pointing_y1_control_lp_filter, pointing_y2_control_lp_filter;
Iir::Butterworth::LowPass<6> opd_lp_filter, opd_control_lp_filter;
const float shear_samplingrate = 12800.;
const float shear_meas_cutoff = 1000.;
const float shear_control_cutoff = 50.;
const float opd_samplingrate = 128000.;
const float opd_meas_cutoff = 1000.;
const float opd_control_cutoff = 100.;

namespace NICEcontrol {

// general
std::atomic<bool> gui_control(true);

// OPD control
float opd_setpoint_gui = 0.0f;           // setpoint entered in GUI, may be out of range
std::atomic<float> opd_setpoint = 0.0f;  // setpoint used in calculation, clipped to valid range
std::atomic<bool> RunOpdControl(false);
std::atomic<float> opd_p = 0.7f;
std::atomic<float> opd_i = 0.0f;
std::atomic<float> opd_dither_freq = 0.0f;
std::atomic<float> opd_dither_amp = 0.0f;

// shear control
float shear_x1_setpoint_gui = 0.0f;
std::atomic<float> shear_x1_setpoint = 0.0f;
float shear_x2_setpoint_gui = 0.0f;
std::atomic<float> shear_x2_setpoint = 0.0f;
float shear_y1_setpoint_gui = 0.0f;
std::atomic<float> shear_y1_setpoint = 0.0f;
float shear_y2_setpoint_gui = 0.0f;
std::atomic<float> shear_y2_setpoint = 0.0f;
std::atomic<bool> RunShearControl(false);
std::atomic<float> shear_p = 0.0f;
std::atomic<float> shear_i = 0.0f;

// pointing control
float pointing_x1_setpoint_gui = 0.0f;
std::atomic<float> pointing_x1_setpoint = 0.0f;
float pointing_x2_setpoint_gui = 0.0f;
std::atomic<float> pointing_x2_setpoint = 0.0f;
float pointing_y1_setpoint_gui = 0.0f;
std::atomic<float> pointing_y1_setpoint = 0.0f;
float pointing_y2_setpoint_gui = 0.0f;
std::atomic<float> pointing_y2_setpoint = 0.0f;
std::atomic<bool> RunPointingControl(false);
std::atomic<float> pointing_p = 0.0f;
std::atomic<float> pointing_i = 0.0f;

// variables that control the measurement thread
std::atomic<bool> RunMeasurement(false);
std::ofstream outputFile;

// initialise opd stage
// MCL_OPDStage opd_stage;
static float opd_open_loop_setpoint = 0.0f;
char serial_number[1024] = "123076463";
PI_E754_Controller opd_stage(serial_number);

// initialise tip/tilt stages
char serial_number1[1024] = "0122040101";
PI_E727_Controller tip_tilt_stage1(serial_number1);

char serial_number2[1024] = "0122042007";
PI_E727_Controller tip_tilt_stage2(serial_number2);

nF_EBD_Controller nF_stage_1("/dev/ttyUSB0");
nF_EBD_Controller nF_stage_2("/dev/ttyUSB1");

void setupActuators() {
  // connect and intialise all piezo stages

  // Tip/tilt stage 1
  tip_tilt_stage1.init();
  // tip_tilt_stage1.autozero(); // run autozero if stage does not move
  tip_tilt_stage1.move_to_x(0.0f);
  tip_tilt_stage1.move_to_y(0.0f);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << "x Position: " << tip_tilt_stage1.readx() << std::endl;
  std::cout << "y Position: " << tip_tilt_stage1.ready() << std::endl;

  // Tip/tilt stage 2
  tip_tilt_stage2.init();
  // tip_tilt_stage2.autozero(); // run autozero if stage does not move
  tip_tilt_stage2.move_to_x(0.0f);
  tip_tilt_stage2.move_to_y(0.0f);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << "x Position: " << tip_tilt_stage2.readx() << std::endl;
  std::cout << "y Position: " << tip_tilt_stage2.ready() << std::endl;

  // nF tip/tilt stages
  nF_stage_1.init();
  nF_stage_1.move_to(0.0, 0.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << "nF Stage 1 Position: " << nF_stage_1.read_x() << ", " << nF_stage_1.read_y() << std::endl;

  nF_stage_2.init();
  nF_stage_2.move_to(0.0, 0.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << "nF Stage 2 Position: " << nF_stage_2.read_x() << ", " << nF_stage_2.read_y() << std::endl;

  // OPD stage
  opd_stage.init();
  opd_stage.move_to(0.0f);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << "OPD Position: " << opd_stage.read() << std::endl;

}

static float shear_x1_ol_setpoint = 0.0f;
static float shear_x2_ol_setpoint = 0.0f;
static float shear_y1_ol_setpoint = 0.0f;
static float shear_y2_ol_setpoint = 0.0f;

static float pointing_x1_ol_setpoint = 0.0f;
static float pointing_x2_ol_setpoint = 0.0f;
static float pointing_y1_ol_setpoint = 0.0f;
static float pointing_y2_ol_setpoint = 0.0f;

TSQueue<Measurement> opdQueue;
TSQueue<Measurement> shear_x1Queue;
TSQueue<Measurement> shear_x2Queue;
TSQueue<Measurement> shear_y1Queue;
TSQueue<Measurement> shear_y2Queue;

TSQueue<Measurement> point_x1Queue;
TSQueue<Measurement> point_x2Queue;
TSQueue<Measurement> point_y1Queue;
TSQueue<Measurement> point_y2Queue;

TSQueue<MeasurementT<int, int>> adc_queues[10];
TSQueue<MeasurementT<int, int>> shear_sum_queue, point_sum_queue;

TSQueue<ControlData> opd_controlData_queue;

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

void run_calculation() {
  int sockfd = setup_ethernet();

  // Open the output file
  std::ofstream outputFile("data.csv");
  if (!outputFile) {
    std::cerr << "Failed to open output file." << std::endl;
  }

  // Write header of the CSV file
  outputFile << "Time (s),OPD (nm)\n";

  int count = 0;
  int buffer_size = 1024;
  char buffer[buffer_size];

  // initialise filters
  shear_x1_lp_filter.setup(shear_samplingrate, shear_meas_cutoff);
  shear_x1_control_lp_filter.setup(shear_samplingrate, shear_control_cutoff);
  shear_x2_lp_filter.setup(shear_samplingrate, shear_meas_cutoff);
  shear_x2_control_lp_filter.setup(shear_samplingrate, shear_control_cutoff);
  shear_y1_control_lp_filter.setup(shear_samplingrate, shear_control_cutoff);
  shear_y2_control_lp_filter.setup(shear_samplingrate, shear_control_cutoff);
  opd_lp_filter.setup(opd_samplingrate, opd_meas_cutoff);

  

  while (true) {
    RunMeasurement.wait(false);

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
    int receivedDataInt[20 * 10];
    std::memcpy(receivedDataInt, buffer, sizeof(int) * 20 * 10);

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
    static int adc_opd_ref[10];
    static float opd_nm[10];
    static float shear_x1_um[10];
    static float shear_x2_um[10];
    static float shear_y1_um[10];
    static float shear_y2_um[10];
    static float point_x1_um[10];
    static float point_x2_um[10];
    static float point_y1_um[10];
    static float point_y2_um[10];



    for (int i = 0; i < 10; i++) {
      counter[i] = receivedDataInt[20 * i];
      adc_shear1[i] = receivedDataInt[20 * i + 1];
      adc_shear2[i] = receivedDataInt[20 * i + 2];
      adc_shear3[i] = receivedDataInt[20 * i + 3];
      adc_shear4[i] = receivedDataInt[20 * i + 4];
      adc_point1[i] = receivedDataInt[20 * i + 5];
      adc_point2[i] = receivedDataInt[20 * i + 6];
      adc_point3[i] = receivedDataInt[20 * i + 7];
      adc_point4[i] = receivedDataInt[20 * i + 8];
      adc_sine_ref[i] = receivedDataInt[20 * i + 9];
      adc_opd_ref[i] = receivedDataInt[20 * i + 10];
      opd_nm[i] = - float(receivedDataInt[20 * i + 11]) / (2 * PI * 10000.) * 1550.; // 0.1 mrad -> nm
      shear_x1_um[i] = float(receivedDataInt[20 * i + 12]) / 3000.; // um
      shear_x2_um[i] = float(receivedDataInt[20 * i + 13]) / 3000.; // um
      shear_y1_um[i] = float(receivedDataInt[20 * i + 14]) / 3000.; // um
      shear_y2_um[i] = float(receivedDataInt[20 * i + 15]) / 3000.; // um
      point_x1_um[i] = float(receivedDataInt[20 * i + 16]) / 1000.; // urad
      point_x2_um[i] = float(receivedDataInt[20 * i + 17]) / 1000.; // urad
      point_y1_um[i] = float(receivedDataInt[20 * i + 18]) / 1000.; // urad
      point_y2_um[i] = float(receivedDataInt[20 * i + 19]) / 1000.; // urad
    }

    // filter opd by piping the 10 new measurements through the filter
    float opd;
    for (int i = 0; i < 10; i++) {
      opd = opd_lp_filter.filter(opd_nm[i]);
    }

    float opd_control_measurement;
    for (int i = 0; i < 10; i++) {
      opd_control_measurement = opd_control_lp_filter.filter(opd_nm[i]);
    }


    // enqueue measurement and time
    // coordinate system of quad cell is rotated by 45 degrees, hence the combination of basis vectors
    opdQueue.push({t, opd});
    shear_x1Queue.push({t, shear_y1_um[0] + shear_x1_um[0]});
    shear_y1Queue.push({t, shear_x1_um[0] - shear_y1_um[0]});
    shear_x2Queue.push({t, shear_y2_um[0] + shear_x2_um[0]});
    shear_y2Queue.push({t, shear_x2_um[0] - shear_y2_um[0]});

    point_x1Queue.push({t, point_y1_um[0] + point_x1_um[0]});
    point_y1Queue.push({t, point_x1_um[0] - point_y1_um[0]});
    point_x2Queue.push({t, point_y2_um[0] + point_x2_um[0]});
    point_y2Queue.push({t, point_x2_um[0] - point_y2_um[0]});

    // get and filter shear
    float shear_x1_control_measurement = shear_x1_control_lp_filter.filter(shear_y1_um[0] + shear_x1_um[0]);
    float shear_x2_control_measurement = shear_x2_control_lp_filter.filter(shear_y2_um[0] + shear_x2_um[0]);
    float shear_y1_control_measurement = shear_y1_control_lp_filter.filter(shear_x1_um[0] - shear_y1_um[0]);
    float shear_y2_control_measurement = shear_y2_control_lp_filter.filter(shear_x2_um[0] - shear_y2_um[0]);

    // get and filter pointing
    float pointing_x1_control_measurement = pointing_x1_control_lp_filter.filter(point_y1_um[0] + point_x1_um[0]);
    float pointing_x2_control_measurement = pointing_x2_control_lp_filter.filter(point_y2_um[0] + point_x2_um[0]);
    float pointing_y1_control_measurement = pointing_y1_control_lp_filter.filter(point_x1_um[0] - point_y1_um[0]);
    float pointing_y2_control_measurement = pointing_y2_control_lp_filter.filter(point_x2_um[0] - point_y2_um[0]);

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
    }


    // write to file
    // outputFile << t << "," << opd << "\n";

    // variables for control
    static float opd_error = 0.0f;
    static float opd_error_integral = 0.0f;
    static float opd_control_signal = 0.0f;

    static float shear_x1_error = 0.0f;
    static float shear_x1_error_integral = 0.0f;
    static float shear_x1_control_signal = 0.0f;

    static float shear_x2_error = 0.0f;
    static float shear_x2_error_integral = 0.0f;
    static float shear_x2_control_signal = 0.0f;

    static float shear_y1_error = 0.0f;
    static float shear_y1_error_integral = 0.0f;
    static float shear_y1_control_signal = 0.0f;

    static float shear_y2_error = 0.0f;
    static float shear_y2_error_integral = 0.0f;
    static float shear_y2_control_signal = 0.0f;

    static float pointing_x1_error = 0.0f;
    static float pointing_x1_error_integral = 0.0f;
    static float pointing_x1_control_signal = 0.0f;

    static float pointing_x2_error = 0.0f;
    static float pointing_x2_error_integral = 0.0f;
    static float pointing_x2_control_signal = 0.0f;

    static float pointing_y1_error = 0.0f;
    static float pointing_y1_error_integral = 0.0f;
    static float pointing_y1_control_signal = 0.0f;

    static float pointing_y2_error = 0.0f;
    static float pointing_y2_error_integral = 0.0f;
    static float pointing_y2_control_signal = 0.0f;
  
    // if RunOpdControl is true, calculate the control signal
    if (RunOpdControl.load()) {

      float setpoint = opd_setpoint.load();

      // calculate error
      opd_error = setpoint - opd_control_measurement;

      // calculate integral
      opd_error_integral += opd_i.load() * opd_error;

      // calculate derivative
      // opd_error_derivative = opd_error - opd_error_prev;

      // calculate control signal
      opd_control_signal = opd_p.load() * opd_error + opd_error_integral;

      // calculate dither signal: sine
      float dither_signal = opd_dither_amp.load() * std::sin(2 * PI * opd_dither_freq.load() * t) / 2.; // correct for double pass (retroreflector)

      // calculate dither signal: white noise
      // std::default_random_engine opd_dith_gen(std::random_device{}());
      // std::normal_distribution<double> dist(0.0, opd_dither_amp.load());
      // float dither_signal = dist(opd_dith_gen);
      
      // units: s, nm * 4
      opd_controlData_queue.push({t, setpoint, opd_control_measurement, opd_control_signal, dither_signal});

      // actuate piezo actuator
      opd_stage.move_to((opd_control_signal + dither_signal) * 1e-3); // convert nm to um
    }
    else {
      opd_error_integral = 0.0f;
    }

    if (RunShearControl.load()) {
      // calculate error
      shear_x1_error = shear_x1_setpoint.load() - shear_x1_control_measurement;
      shear_x2_error = shear_x2_setpoint.load() - shear_x2_control_measurement;
      shear_y1_error = shear_y1_setpoint.load() - shear_y1_control_measurement;
      shear_y2_error = shear_y2_setpoint.load() - shear_y2_control_measurement;

      // calculate integral
      shear_x1_error_integral += shear_i.load() * shear_x1_error;
      shear_x2_error_integral += shear_i.load() * shear_x2_error;
      shear_y1_error_integral += shear_i.load() * shear_y1_error;
      shear_y2_error_integral += shear_i.load() * shear_y2_error;

      // calculate derivative
      // x1d_error_derivative = x1d_error - x1d_error_prev;

      // calculate control signal
      shear_x1_control_signal = shear_p.load() * shear_x1_error + shear_x1_error_integral;
      shear_x2_control_signal = shear_p.load() * shear_x2_error + shear_x2_error_integral;
      shear_y1_control_signal = shear_p.load() * shear_y1_error + shear_y1_error_integral;
      shear_y2_control_signal = shear_p.load() * shear_y2_error + shear_y2_error_integral;

      // actuate piezo actuator
      tip_tilt_stage1.move_to_x(shear_x1_control_signal);
      tip_tilt_stage2.move_to_x(shear_x2_control_signal);
      tip_tilt_stage1.move_to_y(shear_y1_control_signal);
      tip_tilt_stage2.move_to_y(shear_y2_control_signal);
    }
    else {
      shear_x1_error_integral = 0.0f;
      shear_x2_error_integral = 0.0f;
      shear_y1_error_integral = 0.0f;
      shear_y2_error_integral = 0.0f;
    }

    if (RunPointingControl.load()){

      // calculate error
      pointing_x1_error = pointing_x1_setpoint.load() - pointing_x1_control_measurement;
      pointing_x2_error = pointing_x2_setpoint.load() - pointing_x2_control_measurement;
      pointing_y1_error = pointing_y1_setpoint.load() - pointing_y1_control_measurement;
      pointing_y2_error = pointing_y2_setpoint.load() - pointing_y2_control_measurement;

      // calculate integral
      pointing_x1_error_integral += pointing_i.load() * pointing_x1_error;
      pointing_x2_error_integral += pointing_i.load() * pointing_x2_error;
      pointing_y1_error_integral += pointing_i.load() * pointing_y1_error;
      pointing_y2_error_integral += pointing_i.load() * pointing_y2_error;

      // calculate control signal
      pointing_x1_control_signal = pointing_p.load() * pointing_x1_error + pointing_x1_error_integral;
      pointing_x2_control_signal = pointing_p.load() * pointing_x2_error + pointing_x2_error_integral;
      pointing_y1_control_signal = pointing_p.load() * pointing_y1_error + pointing_y1_error_integral;
      pointing_y2_control_signal = pointing_p.load() * pointing_y2_error + pointing_y2_error_integral;

      // actuate piezo actuator
      nF_stage_1.move_to(pointing_x1_control_signal, pointing_y1_control_signal);
      nF_stage_2.move_to(pointing_x2_control_signal, pointing_y2_control_signal);
    }
    else {
      pointing_x1_error_integral = 0.0f;
      pointing_x2_error_integral = 0.0f;
      pointing_y1_error_integral = 0.0f;
      pointing_y2_error_integral = 0.0f;
    }
  }
}

void char_opd() {
  std::cout << "Starting OPD characterisation" << std::endl;

  float settling_time = 1.0; // s
  float recording_time = 1.0; // s

  // set control parameters
  opd_setpoint.store(0.0);
  opd_p.store(0.7);
  opd_i.store(0.009);

  // directory to store files in: dire name is opd_date_time (e.g. measurements/opd_2021-09-01_12:00:00)
  auto datetime_string = get_iso_datestring();
  std::string dirname = "measurements/opd_" + datetime_string;
  std::filesystem::create_directory(dirname);

  // OPEN LOOP CHARACTERISATION
  std::cout << "\t OPD open loop" << std::endl;
  // storage file
  std::string filename = dirname + "/opd_open_loop.csv";
  std::ofstream
  file(filename);
  file << "Time (s),OPD (nm)\n";

  // make sure control loop is off
  RunOpdControl.store(false);
  
  // wait settling time
  std::this_thread::sleep_for(std::chrono::seconds(int(settling_time)));

  // flush OPD queue
  while (!opdQueue.isempty()) {
    opdQueue.pop();
  }

  // record for recording time: every 10 ms, flush opd queue and write contents to file
  auto t_start = std::chrono::high_resolution_clock::now();
  while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - t_start).count() < recording_time) {

    // wait 10 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // write to file
    if (!opdQueue.isempty()) {
      int N = opdQueue.size();
      for (int i = 0; i < N; i++) {
        auto m = opdQueue.pop();
        // format: 6 decimals for time, 3 decimals for OPD
        file << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3) << m.value << "\n";
      }
    }
  }
  file.close();

  // CLOSED LOOP CHARACTERISATION
  std::cout << "\t OPD closed loop" << std::endl;
  // storage file
  filename = dirname + "/opd_closed_loop.csv";
  file.open(filename);
  file << "Time (s),OPD (nm)\n";

  // make sure control loop is on
  RunOpdControl.store(true);

  // wait settling time
  std::this_thread::sleep_for(std::chrono::seconds(int(settling_time)));

  // flush OPD queue
  while (!opdQueue.isempty()) {
    opdQueue.pop();
  }

  // record for recording time: every 10 ms, flush opd queue and write contents to file
  t_start = std::chrono::high_resolution_clock::now();
  while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - t_start).count() < recording_time) {

    // wait 10 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // write to file
    if (!opdQueue.isempty()) {
      int N = opdQueue.size();
      for (int i = 0; i < N; i++) {
        auto m = opdQueue.pop();
        // format: 6 decimals for time, 3 decimals for OPD
        file << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3) << m.value << "\n";
      }
    }
  }
  file.close();


  // IMPULSE RESPONSE CHARACTERISATION
  std::cout << "\t OPD impulse response" << std::endl;
  // storage file
  filename = dirname + "/opd_impulse_response.csv";
  file.open(filename);
  file << "Time (s),Setpoint (nm),Measurement (nm),Control signal (nm),Dither signal (nm)\n";

  // settle control loop
  RunOpdControl.store(true);
  std::this_thread::sleep_for(std::chrono::seconds(int(settling_time)));
  opd_setpoint.store(0.0);

  // flush data queues
  while (!opd_controlData_queue.isempty()) {
    opd_controlData_queue.pop();
  }

  // record for recording time: every 100 ms, flush opd queue and write contents to file. Also, toggle the setpoint between 0 and 100
  t_start = std::chrono::high_resolution_clock::now();
  bool setpoint_high = false;
  while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - t_start).count() < recording_time) {

    // wait 100 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // toggle setpoint
    if (setpoint_high) {
      opd_setpoint.store(0.0);
    } else {
      opd_setpoint.store(100.0);
    }
    setpoint_high = !setpoint_high;

    // write to file
    if (!opd_controlData_queue.isempty()) {
      int N = opd_controlData_queue.size();
      for (int i = 0; i < N; i++) {
        auto m = opd_controlData_queue.pop();
        // format: 6 decimals for time, 3 decimals for OPD
        file << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3) << m.setpoint << "," << std::fixed << std::setprecision(3) << m.measurement << "," << std::fixed << std::setprecision(3) << m.control_signal << "," << std::fixed << std::setprecision(3) << m.dither_signal << "\n";
      }
    }

  }
  file.close();



  
  // FREQUENCY CHARACTERISATION
  // for all dither frequencies and amplitudes, run the control loop for a while and record dither and opd measurments
  std::cout << "\t OPD frequency characterisation" << std::endl;

    // dither frequencies
  std::vector<float> dither_freqs = {0.1, 0.5, 1.0, 5.0, 10.0}; // Hz

  // add logscaled frequencies: 100 entries from 10 Hz to 1 kHz
  for (int i = 1; i < 100; i++) {
    dither_freqs.push_back(10.0 * std::pow(10, i / 100.0));
  }

  // dither amplitudes: all 20 nm
  std::vector<float> dither_amps(dither_freqs.size(), 20.0); // nm

  // for 0.1 Hz and 1 Hz, use 100 nm
  // dither_amps[0] = 100.0;
  // dither_amps[1] = 100.0;

  // recording time: at least 1 s, at most 10/freq
  std::vector<float> recording_times(dither_freqs.size(), 0.0); // s
  for (int i = 0; i < dither_freqs.size(); i++) {
    recording_times[i] = std::max(1.0, 10.0 / dither_freqs[i]);
  }

  // seperate storage file for each frequency

  for (int i = 0; i < dither_freqs.size(); i++) {
    std::cout << "\t f = " << dither_freqs[i] << " Hz" << std::endl;
    // storage file: frequency in format 1.2345e67
    filename = dirname + "/opd_freq_" + std::to_string(dither_freqs[i]) + "_hz.csv";
    file.open(filename);
    file << "Time (s),Setpoint (nm),Measurement (nm),Control signal (nm),Dither signal (nm)\n";

    // set control parameters
    opd_setpoint.store(0.0);

    // set dither loop parameters
    opd_dither_freq.store(dither_freqs[i]);
    opd_dither_amp.store(dither_amps[i]);

    // settle control loop
    RunOpdControl.store(true);
    std::this_thread::sleep_for(std::chrono::seconds(int(settling_time)));
    

    // flush data queues
    while (!opd_controlData_queue.isempty()) {
      opd_controlData_queue.pop();
    }

    // record for recording time: every 100 ms, flush opd queue and write contents to file.
    t_start = std::chrono::high_resolution_clock::now();
    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - t_start).count() < recording_times[i]) {

      // wait 100 ms
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      // write to file
      if (!opd_controlData_queue.isempty()) {
        int N = opd_controlData_queue.size();
        for (int i = 0; i < N; i++) {
          auto m = opd_controlData_queue.pop();
          // format: 6 decimals for time, 3 decimals for OPD
          file << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(3) << m.setpoint << "," << std::fixed << std::setprecision(3) << m.measurement << "," << std::fixed << std::setprecision(3) << m.control_signal << "," << std::fixed << std::setprecision(3) << m.dither_signal << "\n";
        }
      }

    }
    file.close();
  }



  // DONE
  gui_control.store(true);
  std::cout << "Finished OPD characterisation" << std::endl;
}

void startMeasurement() {
  RunMeasurement.store(true);
  RunMeasurement.notify_all();
}

void stopMeasurement() { RunMeasurement.store(false); }

void RenderUI() {
  ImGui::Begin("NICE Control");

  ImGuiIO &io = ImGui::GetIO();

  // opd control gui parameters
  static int gui_opd_loop_select = 0;
  static float opd_p_gui = 0.700f;
  static float opd_i_gui = 0.009f;
  static float opd_dither_freq_gui = 0.0f;
  static float opd_dither_amp_gui = 0.0f;

  // shear control gui parameters
  static int shear_loop_select = 0;
  static float shear_p_gui = 0.4f;
  static float shear_i_gui = 0.007f;
  shear_p.store(shear_p_gui);
  shear_i.store(shear_i_gui);

  // pointing control gui parameters
  static int pointing_loop_select = 0;
  static float pointing_p_gui = 1e-5f;
  static float pointing_i_gui = 1e-7f;
  pointing_p.store(pointing_p_gui);
  pointing_i.store(pointing_i_gui);

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

  // ADC measurements
  if (ImGui::CollapsingHeader("ADC Measurements")) {

    static ScrollingBufferT<int, int> adc_buffers[10];
    static ScrollingBufferT<int, int> shear_sum_buffer, point_sum_buffer;
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
      float mean = std::accumulate(last_1000_shear_sum.begin(), last_1000_shear_sum.end(), 0.0) /
                   last_1000_shear_sum.size();
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
      float mean = std::accumulate(last_1000_point_sum.begin(), last_1000_point_sum.end(), 0.0) /
                   last_1000_point_sum.size();
      // print peak-to-peak as floating point number, e.g. 9.432E4
      ImGui::Text("Peak-to-peak PointSum: %.2E", float(max - min));
      ImGui::Text("Mean PointSum: %.2E", mean);
    }


    static float adc_history_length = 1000.f;
    ImGui::SliderFloat("ADC History", &adc_history_length, 1, 128000, "%.5f s", ImGuiSliderFlags_Logarithmic);

    

    // plot label names
    static const char *plot_labels[10] = {"Shear UP", "Shear LEFT", "Shear RIGHT", "Shear DOWN",
                                          "Point UP", "Point LEFT", "Point RIGHT", "Point DOWN",
                                          "SineRef", "OPDRef"};

    // plot style
    static float thickness = 5;
    static ImPlotAxisFlags xflags = ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoTickLabels;
    static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;

    if (ImPlot::BeginPlot("##ADC", ImVec2(-1, 400 * io.FontGlobalScale))) {
      ImPlot::SetupAxes(nullptr, nullptr, xflags, yflags);
      ImPlot::SetupAxisLimits(ImAxis_X1, t_adc - adc_history_length, t_adc, ImGuiCond_Always);
      ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
      ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
      for (int i = 0; i < 10; i++) {
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(i), thickness);
        ImPlot::PlotLine(plot_labels[i], &adc_buffers[i].Data[0].time, &adc_buffers[i].Data[0].value, adc_buffers[i].Data.size(),
                         0, adc_buffers[i].Offset, 2 * sizeof(int));
      }

      // shear sum
      ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(10), thickness);
      ImPlot::PlotLine("ShearSum", &shear_sum_buffer.Data[0].time, &shear_sum_buffer.Data[0].value,
                       shear_sum_buffer.Data.size(), 0, shear_sum_buffer.Offset, 2 * sizeof(int));

      // pointing sum
      ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(11), thickness);
      ImPlot::PlotLine("PointSum", &point_sum_buffer.Data[0].time, &point_sum_buffer.Data[0].value,
                       point_sum_buffer.Data.size(), 0, point_sum_buffer.Offset, 2 * sizeof(int));
      ImPlot::EndPlot();
    }
  }


  if (ImGui::CollapsingHeader("OPD")) {
    // characterise button: launch characterisation of OPD thread and deactivate gui control
    static bool opd_char_button = false;
    
    ImGui::Checkbox("Characterise OPD", &opd_char_button);

    // print status of gui_control
    ImGui::Text("GUI control: %s", gui_control.load() ? "true" : "false");

    if (opd_char_button) {
      gui_control.store(false);
      opd_char_button = false;
      // std::jthread char_opd_thread(char_opd);
      char_opd();
    }

    // control mode selector
    ImGui::Text("Control mode:");
    ImGui::SameLine();
    ImGui::RadioButton("Off##OPD", &gui_opd_loop_select, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Open loop##OPD", &gui_opd_loop_select, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Closed loop##OPD", &gui_opd_loop_select, 2);

    
    if (gui_control.load()) {
      // run control if "Closed loop" is selected
      if (gui_opd_loop_select == 2) {RunOpdControl.store(true);}
      else {RunOpdControl.store(false);}

      // move stage to open loop setpoint if "Open loop" is selected
      if (gui_opd_loop_select == 1) {opd_stage.move_to(opd_open_loop_setpoint);}

      // store control loop parameters
      opd_setpoint.store(opd_setpoint_gui);
      opd_p.store(opd_p_gui);
      opd_i.store(opd_i_gui);
      opd_dither_freq.store(opd_dither_freq_gui);
      opd_dither_amp.store(opd_dither_amp_gui);
    }

    

    // real time plot
    static ScrollingBuffer opd_buffer, setpoint_buffer;
    static ScrollingBuffer opd_dith_buffer;

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

    // get control signals
    if (!opd_controlData_queue.isempty()) {
      while (!opd_controlData_queue.isempty()) {
        auto m = opd_controlData_queue.pop();
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
      ImPlot::SetupAxisLimits(ImAxis_Y1, -opd_dither_amp.load(), opd_dither_amp.load());
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
      // Display measurement
      ImGui::Text("Current measurement: %.4f", current_measurement);

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

      float piezo_measurement = 0.0f;

      // Display measurement
      ImGui::Text("Current measurement: %.4f", piezo_measurement);

      // open loop setpoint µm
      ImGui::SliderFloat("Setpoint##OPD", &opd_open_loop_setpoint, 0.f, 15.0f);

      ImGui::TreePop();
    }

    if (ImGui::TreeNode("Control loop##OPD")) {
      ImGui::Text("Control loop parameters:");

      // sliders for p ,i
      ImGui::SliderFloat("P##OPD", &opd_p_gui, 1e-4f, 1e2f, "%.5f", ImGuiSliderFlags_Logarithmic);
      ImGui::SliderFloat("I##OPD", &opd_i_gui, 1e-4f, 3e-1f, "%.5f", ImGuiSliderFlags_Logarithmic);

      const float opd_setpoint_min = -1000.0f, opd_setpoint_max = 1000.0f;

      // opd input: drag
      ImGui::SliderFloat("(Drag or double-click to adjust)", &opd_setpoint_gui, opd_setpoint_min, opd_setpoint_max,
                         "%.1f nm", ImGuiSliderFlags_AlwaysClamp);

      // clamp opd_setpoint_gui to min/max
      if (opd_setpoint_gui < opd_setpoint_min) opd_setpoint_gui = opd_setpoint_min;
      if (opd_setpoint_gui > opd_setpoint_max) opd_setpoint_gui = opd_setpoint_max;

      // dither parameters
      ImGui::SliderFloat("Dither frequency##OPD", &opd_dither_freq_gui, 0.1f, 1000.0f, "%.2f Hz", ImGuiSliderFlags_Logarithmic);
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

    // run control if "Closed loop" is selected
    if (shear_loop_select == 2) {
      RunShearControl.store(true);
    } else {
      RunShearControl.store(false);
    }

    // open loop
    if (shear_loop_select == 1) {
      // actuators are rotated 90 degrees, hence the x/y swap
      tip_tilt_stage1.move_to_x(shear_x1_ol_setpoint);
      tip_tilt_stage1.move_to_y(shear_y1_ol_setpoint);
      tip_tilt_stage2.move_to_x(shear_x2_ol_setpoint);
      tip_tilt_stage2.move_to_y(shear_y2_ol_setpoint);
    }

    // real time plot
    static ScrollingBuffer shear_x1_buffer, shear_x2_buffer, shear_y1_buffer, shear_y2_buffer;

    static float t_gui_x = 0;

    // if measurement is running, update gui time.
    if (RunMeasurement.load()) {
      t_gui_x = getTime();
    }

    // add the entire MeasurementQueue to the buffer
    if (!shear_x1Queue.isempty()) {
      while (!shear_x1Queue.isempty()) {
        auto m = shear_x1Queue.pop();
        shear_x1_buffer.AddPoint(m.time, m.value);
      }
    }

    if (!shear_x2Queue.isempty()) {
      while (!shear_x2Queue.isempty()) {
        auto m = shear_x2Queue.pop();
        shear_x2_buffer.AddPoint(m.time, m.value);
      }
    }

    if (!shear_y1Queue.isempty()) {
      while (!shear_y1Queue.isempty()) {
        auto m = shear_y1Queue.pop();
        shear_y1_buffer.AddPoint(m.time, m.value);
      }
    }

    if (!shear_y2Queue.isempty()) {
      while (!shear_y2Queue.isempty()) {
        auto m = shear_y2Queue.pop();
        shear_y2_buffer.AddPoint(m.time, m.value);
      }
    }

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
      static ImVec4 color = ImVec4(1, 1, 0, 1);
      static ScrollingBuffer tip_tilt_actuator_buffer;
      tip_tilt_actuator_buffer.AddPoint(t_gui_x, tip_tilt_stage1.readx());

      if (ImPlot::BeginPlot("##Tip/Tilt actuator", ImVec2(-1, 150 * io.FontGlobalScale))) {
        ImPlot::SetupAxes(nullptr, nullptr, x1_xflags, x1_yflags);
        ImPlot::SetupAxisLimits(ImAxis_X1, t_gui_x - x1_history_length, t_gui_x, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
        ImPlot::SetNextLineStyle(color, x1_thickness);
        ImPlot::PlotLine("Tip/Tilt actuator position", &tip_tilt_actuator_buffer.Data[0].x,
                         &tip_tilt_actuator_buffer.Data[0].y, tip_tilt_actuator_buffer.Data.size(), 0,
                         tip_tilt_actuator_buffer.Offset, 2 * sizeof(float));
        ImPlot::EndPlot();
      }

      float tip_tilt_actuator_measurement = 0.0f;

      // Display measurement
      ImGui::Text("Current measurement: %.4f", tip_tilt_actuator_measurement);

      // open loop setpoint µm
      ImGui::SliderFloat("X1 setpoint##Shear", &shear_x1_ol_setpoint, -1000.0f, 1000.0f);
      ImGui::SliderFloat("Y1 setpoint##Shear", &shear_y1_ol_setpoint, -1000.0f, 1000.0f);
      ImGui::SliderFloat("X2 setpoint##Shear", &shear_x2_ol_setpoint, -1000.0f, 1000.0f);
      ImGui::SliderFloat("Y2 setpoint##Shear", &shear_y2_ol_setpoint, -1000.0f, 1000.0f);

      ImGui::TreePop();
    }

    // control
    if (ImGui::TreeNode("Control loop##X1D")) {
      ImGui::Text("Control loop parameters:");

      // sliders for p ,i
      ImGui::SliderFloat("P##X1D", &shear_p_gui, 0.0f, 1.0f);
      ImGui::SliderFloat("I##X1D", &shear_i_gui, 0.0f, 0.05f);

      const float x1d_setpoint_min = -200.0f, x1d_setpoint_max = 200.0f;

      // x1d input: drag
      ImGui::SliderFloat("(Drag or double-click to adjust)", &shear_x1_setpoint_gui, x1d_setpoint_min, x1d_setpoint_max,
                         "%.1f nm", ImGuiSliderFlags_AlwaysClamp);

      // clamp shear_x1_setpoint_gui to min/max
      if (shear_x1_setpoint_gui < x1d_setpoint_min) shear_x1_setpoint_gui = x1d_setpoint_min;
      if (shear_x1_setpoint_gui > x1d_setpoint_max) shear_x1_setpoint_gui = x1d_setpoint_max;

      // set shear_x1_setpoint
      shear_x1_setpoint.store(shear_x1_setpoint_gui);

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

    // run control if "Closed loop" is selected
    if (pointing_loop_select == 2) {
      RunPointingControl.store(true);
    } else {
      RunPointingControl.store(false);
    }

    // open loop
    if (pointing_loop_select == 1) {
      nF_stage_1.move_to(pointing_x1_ol_setpoint, pointing_y1_ol_setpoint);
      nF_stage_2.move_to(pointing_x2_ol_setpoint, pointing_y2_ol_setpoint);
    }




    // real time plot
    static ScrollingBuffer point_x1_buffer, point_x2_buffer, point_y1_buffer, point_y2_buffer;

    static float t_gui_x = 0;

    // if measurement is running, update gui time.
    if (RunMeasurement.load()) {
      t_gui_x = getTime();
    }

    // add the entire MeasurementQueue to the buffer
    if (!point_x1Queue.isempty()) {
      while (!point_x1Queue.isempty()) {
        auto m = point_x1Queue.pop();
        point_x1_buffer.AddPoint(m.time, m.value);
      }
    }

    if (!point_x2Queue.isempty()) {
      while (!point_x2Queue.isempty()) {
        auto m = point_x2Queue.pop();
        point_x2_buffer.AddPoint(m.time, m.value);
      }
    }

    if (!point_y1Queue.isempty()) {
      while (!point_y1Queue.isempty()) {
        auto m = point_y1Queue.pop();
        point_y1_buffer.AddPoint(m.time, m.value);
      }
    }

    if (!point_y2Queue.isempty()) {
      while (!point_y2Queue.isempty()) {
        auto m = point_y2Queue.pop();
        point_y2_buffer.AddPoint(m.time, m.value);
      }
    }

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
      ImPlot::PlotLine("Pointing X1", &point_x1_buffer.Data[0].x, &point_x1_buffer.Data[0].y, point_x1_buffer.Data.size(),
                       0, point_x1_buffer.Offset, 2 * sizeof(float));
      ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(1), pointing_thickness);
      ImPlot::PlotLine("Pointing Y1", &point_y1_buffer.Data[0].x, &point_y1_buffer.Data[0].y, point_y1_buffer.Data.size(),
                       0, point_y1_buffer.Offset, 2 * sizeof(float));
      ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(2), pointing_thickness);
      ImPlot::PlotLine("Pointing X2", &point_x2_buffer.Data[0].x, &point_x2_buffer.Data[0].y, point_x2_buffer.Data.size(),
                       0, point_x2_buffer.Offset, 2 * sizeof(float));
      ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(3), pointing_thickness);
      ImPlot::PlotLine("Pointing Y2", &point_y2_buffer.Data[0].x, &point_y2_buffer.Data[0].y, point_y2_buffer.Data.size(),
                       0, point_y2_buffer.Offset, 2 * sizeof(float));
      ImPlot::EndPlot();
    }

    // pointing actuator
    if (ImGui::TreeNode("Actuator##Pointing")) {

      // open loop setpoint µm
      ImGui::SliderFloat("X1 setpoint##Pointing", &pointing_x1_ol_setpoint, -5.0f, 5.0f);
      ImGui::SliderFloat("Y1 setpoint##Pointing", &pointing_y1_ol_setpoint, -5.0f, 5.0f);
      ImGui::SliderFloat("X2 setpoint##Pointing", &pointing_x2_ol_setpoint, -5.0f, 5.0f);
      ImGui::SliderFloat("Y2 setpoint##Pointing", &pointing_y2_ol_setpoint, -5.0f, 5.0f);

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
      ImGui::SliderFloat("(Drag or double-click to adjust)", &pointing_x1_setpoint_gui, pointing_setpoint_min, pointing_setpoint_max,
                         "%.1f nm", ImGuiSliderFlags_AlwaysClamp);

      // set pointing_x1_setpoint
      pointing_x1_setpoint.store(pointing_x1_setpoint_gui);

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
  {
    NICEcontrol::RunMeasurement.store(false);
  }
  computeThread.join();

  return 0;
}
