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
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <mutex>
#include <numbers>
#include <queue>
#include <random>
#include <thread>

// ethernet
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

// IIR filter from https://github.com/berndporr/iir1
#include <cstring>

#include "../lib/fonts/SourceSans3Regular.cpp"
#include "../lib/implot/implot.h"
#include "Iir.h"

// Actuators
#include "MCL_NanoDrive.hpp"       // Controller for MCL OPD Stage
#include "PI_E727_Controller.hpp"  // Controller for PI Tip/Tilt Stages
#include "PI_E754_Controller.hpp"  // Controller for PI OPD Stage
#include "nF_EBD_Controller.hpp"   // Controller for nanoFaktur Tip/Tilt Stages

// Data types
#include "ControlData.hpp"
#include "MeasurementT.hpp"
#include "ScrollingBuffer.hpp"
#include "ScrollingBufferT.hpp"
#include "SensorData.hpp"
#include "TSCircularBuffer.hpp"

// Controllers (PID etc.)
#include "Controllers.hpp"
#include "DiagPIController.hpp"
#include "MIMOControlLoop.hpp"
#include "SISOControlLoop.hpp"

// functions
#include "characterise_control_loop.hpp"
#include "characterise_joint_closed_loop.hpp"
#include "utils.hpp"

// FFT for data streams
#include "FftCalculator.hpp"

// NullLockin
#include "NullLockin.hpp"

// EtherCAT UDP Interface
#include "EthercatUdpInterface.hpp"

// Tango
#include "camera_if.hpp"

// white noise for dithering
#include <random>

// format datetime for logging
#include <ctime>

// Windows
#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

static void glfw_error_callback(int error, const char *description) {
  fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

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
std::atomic<bool> reset_phase_unwrap(false);

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
// PI_E754_Controller opd_stage(serial_number);

// initialise tip/tilt stages
char serial_number1[1024] = "0122040101";
PI_E727_Controller tip_tilt_stage1(serial_number1);

char serial_number2[1024] = "0122042007";
PI_E727_Controller tip_tilt_stage2(serial_number2);

nF_EBD_Controller nF_stage_1("/dev/ttyUSB2");
nF_EBD_Controller nF_stage_2("/dev/ttyUSB3");

void setupActuators() {
  // connect and intialise all piezo stages

  // // Tip/tilt stage 1
  // tip_tilt_stage1.init();
  // // tip_tilt_stage1.autozero(); // run autozero if stage does not move
  // tip_tilt_stage1.move_to_x(0.0f);
  // tip_tilt_stage1.move_to_y(0.0f);
  // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // std::cout << "\tPosition: (" << tip_tilt_stage1.readx() << ", " << tip_tilt_stage1.ready() << ") urad" <<
  // std::endl;

  // // Tip/tilt stage 2
  // tip_tilt_stage2.init();
  // // tip_tilt_stage2.autozero(); // run autozero if stage does not move
  // tip_tilt_stage2.move_to_x(0.0f);
  // tip_tilt_stage2.move_to_y(0.0f);
  // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // std::cout << "\tPosition: (" << tip_tilt_stage2.readx() << ", " << tip_tilt_stage2.ready() << ") urad" <<
  // std::endl;

  // // nF tip/tilt stages
  // nF_stage_1.init();
  // nF_stage_1.move_to({0.0, 0.0});
  // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // auto pos = nF_stage_1.read();
  // std::cout << "nF Stage 1 Position: " << pos[0] << ", " << pos[1] << std::endl;

  // nF_stage_2.init();
  // nF_stage_2.move_to({0.0, 0.0});
  // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // pos = nF_stage_2.read();
  // std::cout << "nF Stage 2 Position: " << pos[0] << ", " << pos[1] << std::endl;

  // OPD stage
  // opd_stage.init();
  // opd_stage.move_to(0.f);
  // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // std::cout << "\tPosition: " << opd_stage.read() << " nm" << std::endl;
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
// SISOControlLoop opd_loop(opd_controller, opd_stage);
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
    auto t = utils::getTime();

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
    const static int num_channels = 22;
    const static int num_timepoints = 10;
    static int prev_counter = 0;      // to check for gaps in data
    static int prev_gap_counter = 0;  // to calculate distance between gaps
    int receivedDataInt[num_channels * num_timepoints];
    std::memcpy(receivedDataInt, buffer, sizeof(int) * num_channels * num_timepoints);

    static int counter[num_timepoints];
    static int adc_shear1[num_timepoints];
    static int adc_shear2[num_timepoints];
    static int adc_shear3[num_timepoints];
    static int adc_shear4[num_timepoints];
    static int adc_point1[num_timepoints];
    static int adc_point2[num_timepoints];
    static int adc_point3[num_timepoints];
    static int adc_point4[num_timepoints];
    static int adc_sine_ref[num_timepoints];
    static int adc_sci_null[num_timepoints];
    static int adc_sci_mod[num_timepoints];
    static int adc_opd_ref[num_timepoints];
    static float opd_rad[num_timepoints];
    static int32_t opd_int[num_timepoints];
    static float shear_x1_um[num_timepoints];
    static float shear_x2_um[num_timepoints];
    static float shear_y1_um[num_timepoints];
    static float shear_y2_um[num_timepoints];
    static float point_x1_um[num_timepoints];
    static float point_x2_um[num_timepoints];
    static float point_y1_um[num_timepoints];
    static float point_y2_um[num_timepoints];
    static float opd_rad_i_prev = 0.0f;

    for (int i = 0; i < num_timepoints; i++) {
      counter[i] = receivedDataInt[num_channels * i];

      // check for gaps in data
      if (counter[i] - prev_counter > 1) {
        int n_missing_samples = counter[i] - prev_counter - 1;
        int n_samples_between_gaps = counter[i] - prev_gap_counter - 1;

        std::cout << "Gap in data: " << prev_counter << " -> " << counter[i] << " (" << n_missing_samples
                  << " missing samples) " << n_samples_between_gaps << " samples between gaps" << std::endl;
        prev_gap_counter = counter[i];
      }

      // print current counter
      // std::cout << "Counter: " << counter[i] << std::endl;

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
      opd_rad[i] =
          -float(opd_int[i]) * std::numbers::pi / (std::pow(2.0, 23) - 1.);    // phase (signed 24 bit int) -> rad
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

      prev_counter = counter[i];
    }

    // print first and last counter of the package
    // std::cout << "Counter: " << counter[0] << " -> " << counter[num_timepoints - 1] << std::endl;

    // phase-unwrap the OPD signal
    // has to be done before filtering, since filtering smoothes out the jumps
    const int max_unwrap_iters = 500;
    static int unwrap_iters = 0;

    for (int i = 0; i < num_timepoints; i++) {
      if (reset_phase_unwrap.load()) {
        unwrap_iters = 0;
        opd_rad_i_prev = 0;
      }

      opd_rad[i] = opd_rad[i] + unwrap_iters * 2 * std::numbers::pi;  // add the current number of unwrapping iterations
      while ((opd_rad[i] - opd_rad_i_prev > std::numbers::pi) && (unwrap_iters > -max_unwrap_iters)) {
        opd_rad[i] -= 2 * std::numbers::pi;
        unwrap_iters--;
      }
      while ((opd_rad[i] - opd_rad_i_prev < -std::numbers::pi) && (unwrap_iters < max_unwrap_iters)) {
        opd_rad[i] += 2 * std::numbers::pi;
        unwrap_iters++;
      }
      opd_rad_i_prev = opd_rad[i];
    }

    // filter signals
    for (int i = 0; i < num_timepoints; i++) {
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
    opd_nm_f = opd_rad_f * 1550 / (2 * std::numbers::pi);

    // enqueue sensor data
    sensorDataQueue.push(
        {t, opd_nm_f, shear_x1_f, shear_x2_f, shear_y1_f, shear_y2_f, point_x1_f, point_x2_f, point_y1_f, point_y2_f});

    sci_null_queue.push({t, sci_null});

    // enqueue adc measurements
    for (int i = 0; i < num_timepoints; i++) {
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
    // opd_loop.control(t, opd_nm_f);
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

void startMeasurement() {
  RunMeasurement.store(true);
  RunMeasurement.notify_all();
}

void stopMeasurement() { RunMeasurement.store(false); }

void RenderUI() {
  ImGui::Begin("NICE Control");

  ImGuiIO &io = ImGui::GetIO();

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
    // if (gui_opd_loop_select == 2) {
    //   opd_loop.control_mode.store(4);
    // } else if (gui_opd_loop_select == 1) {
    //   opd_loop.control_mode.store(1);
    // } else {
    //   opd_loop.control_mode.store(0);
    // }

    // opd_loop.setpoint.store(opd_setpoint_gui);
    // opd_loop.p.store(opd_p_gui);
    // opd_loop.i.store(opd_i_gui);
    // opd_loop.dither_freq.store(opd_dither_freq_gui);
    // opd_loop.dither_amp.store(opd_dither_amp_gui);

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

      // TODO: if saving data, write every 12800th sample to file
      if (recording_running) {
        file << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(2) << m.opd
             << "\n";
      }
    }
  }

  // add sci null measurements to the plot buffer
  static int sci_null_sample_no = 0;
  if (!sci_null_queue.isempty()) {
    int N = sci_null_queue.size();
    for (int i = 0; i < N; i++) {
      auto m = sci_null_queue.pop();
      sci_null_buffer.AddPoint(m.time, m.value);

      // write to file
      if (record_sci_null && sci_null_sample_no % 12800 == 0) {  // write every 12800th sample
        sci_null_file << std::fixed << std::setprecision(6) << m.time << "," << std::fixed << std::setprecision(1)
                      << m.value << "\n";
      }
      sci_null_sample_no++;
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

  static float t_gui = 0;
  t_gui = utils::getTime();

  // science camera
  if (ImGui::CollapsingHeader("Science Camera")) {
    init_camera();
    int size = get_size();
    std::cout << "Image sise: " << size << std::endl;

    std::vector<unsigned short> image;
    image = get_image();

    // print 5 elements of the image
    std::cout << "Image: ";
    for (int i = 0; i < 5; i++) {
      std::cout << image[i] << " ";
    }
    std::cout << std::endl;
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
    // if (!adc_buffers[9].Data.empty()) {
    //   auto last_1000_measurements_data = adc_buffers[9].GetLastN(1000);
    //   std::vector<int> last_1000_measurements;
    //   for (auto &m : last_1000_measurements_data) {
    //     last_1000_measurements.push_back(m.value);
    //   }
    //   int min = *std::min_element(last_1000_measurements.begin(), last_1000_measurements.end());
    //   int max = *std::max_element(last_1000_measurements.begin(), last_1000_measurements.end());
    //   float mean = std::accumulate(last_1000_measurements.begin(), last_1000_measurements.end(), 0.0) /
    //                last_1000_measurements.size();
    //   // print peak-to-peak as floating point number, e.g. 9.432E4
    //   ImGui::Text("Peak-to-peak OPDRef: %.2E", float(max - min));
    //   ImGui::Text("Mean OPDRef: %.2E", mean);
    // }

    // same for shear sum
    // if (!adc_buffers[0].Data.empty()) {
    //   auto last_1000_shear_sum_data = shear_sum_buffer.GetLastN(1000);
    //   std::vector<int> last_1000_shear_sum;
    //   for (auto &m : last_1000_shear_sum_data) {
    //     last_1000_shear_sum.push_back(m.value);
    //   }
    //   int min = *std::min_element(last_1000_shear_sum.begin(), last_1000_shear_sum.end());
    //   int max = *std::max_element(last_1000_shear_sum.begin(), last_1000_shear_sum.end());
    //   float mean =
    //       std::accumulate(last_1000_shear_sum.begin(), last_1000_shear_sum.end(), 0.0) / last_1000_shear_sum.size();
    //   // print peak-to-peak as floating point number, e.g. 9.432E4
    //   ImGui::Text("Peak-to-peak ShearSum: %.2E", float(max - min));
    //   ImGui::Text("Mean ShearSum: %.2E", mean);
    // }

    // same for pointing sum
    // if (!adc_buffers[4].Data.empty()) {
    //   auto last_1000_point_sum_data = point_sum_buffer.GetLastN(1000);
    //   std::vector<int> last_1000_point_sum;
    //   for (auto &m : last_1000_point_sum_data) {
    //     last_1000_point_sum.push_back(m.value);
    //   }
    //   int min = *std::min_element(last_1000_point_sum.begin(), last_1000_point_sum.end());
    //   int max = *std::max_element(last_1000_point_sum.begin(), last_1000_point_sum.end());
    //   float mean =
    //       std::accumulate(last_1000_point_sum.begin(), last_1000_point_sum.end(), 0.0) / last_1000_point_sum.size();
    //   // print peak-to-peak as floating point number, e.g. 9.432E4
    //   ImGui::Text("Peak-to-peak PointSum: %.2E", float(max - min));
    //   ImGui::Text("Mean PointSum: %.2E", mean);
    // }

    static float adc_history_length = 1000.f;
    ImGui::SliderFloat("ADC History", &adc_history_length, 1, 128000, "%.5f s", ImGuiSliderFlags_Logarithmic);

    // plot label names
    static const char *plot_labels[10] = {"Shear UP",   "Shear LEFT",  "Shear RIGHT", "Shear DOWN", "Point UP",
                                          "Point LEFT", "Point RIGHT", "Point DOWN",  "SineRef",    "OPDRef"};

    // plot style
    static float thickness = 2;
    static ImPlotAxisFlags xflags = ImPlotAxisFlags_NoTickLabels;
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
    // history length slider
    static float sci_null_history_length = 5.0;
    ImGui::SliderFloat("Sci_null History", &sci_null_history_length, 1e-3, 50., "%.3f s", ImGuiSliderFlags_Logarithmic);

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
      ImPlot::PlotLine("Sci_null", &sci_null_buffer.Data[0].x, &sci_null_buffer.Data[0].y, sci_null_buffer.Data.size(),
                       0, sci_null_buffer.Offset, 2 * sizeof(int));
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
        reset_phase_unwrap.store(1);
      } else {
        reset_phase_unwrap.store(0);
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
      static ImPlotAxisFlags fft_yflags = ImPlotAxisFlags_None;  // ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
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
      static float opd_i_ethercat = 0.0f;
      ImGui::SliderFloat("P##OPD EtherCAT", &opd_p_ethercat, 1e-4f, 1e0f, "%.5f", ImGuiSliderFlags_Logarithmic);
      ImGui::SliderFloat("I##OPD EtherCAT", &opd_i_ethercat, 1e-1f, 1e3f, "%.5f", ImGuiSliderFlags_Logarithmic);

      // two binary flags: Reset phase unwrap, and run control loop
      static bool reset_phase_unwrap = false;
      static bool run_control_loop = false;
      ImGui::Checkbox("Reset phase unwrap", &reset_phase_unwrap);

      // run control loop defined via gui_opd_loop_select
      if (gui_opd_loop_select == 0) {
        run_control_loop = false;
      } else if (gui_opd_loop_select == 1) {
        run_control_loop = true;
      } else {
        run_control_loop = false;
      }

      // Interface to EtherCAT master: Send a few bytes via UDP to the master receiver
      // 5 bytes data via UDP:
      // 4 bytes: OPD setpoint float
      // 1 byte: flags: X X X X X X reset_phase_unwrap run_control_loop
      const int udp_port = 8888;
      const char *udp_ip = "192.168.88.177";
      static EthercatUdpInterface ec_udp_if(udp_ip, udp_port);
      ec_udp_if.send_commands(opd_setpoint_ethercat * 1.0e-3, opd_p_ethercat, opd_i_ethercat, reset_phase_unwrap,
                              run_control_loop);

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
