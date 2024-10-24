#pragma once

#include <atomic>
#include <cmath>
#include <numbers>

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
    float dither_signal = this->dither_amp.load() * std::sin(2 * std::numbers::pi * this->dither_freq.load() * t);

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