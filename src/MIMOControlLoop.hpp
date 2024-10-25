#pragma once

#include <array>
#include <atomic>
#include <cmath>
#include <numbers>

template <int N, class C, class A>  // N x N MIMO control loop
class MIMOControlLoop {
 public:
  // control mode is 0 by default
  MIMOControlLoop(C &controller, A &actuator) : controller(controller), actuator(actuator) {}

  void control(double t, std::array<float, N> sensor_data) {
    std::array<float, N> controller_input = {0.0f};
    std::array<float, N> controller_output = {0.0f};
    std::array<double, N> actuator_command = {0.0f};

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
    dither_signal[dither_axis_loc] = dither_amp_loc * std::sin(2 * std::numbers::pi * dither_freq_loc * t);

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

  C &controller;
  A &actuator;

  // data buffer: N x controldata
  // TSCircularBuffer<ControlDataN<N>> controlDataBuffer;
};