#pragma once

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