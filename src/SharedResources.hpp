#pragma once

#include <array>
#include <atomic>
#include <string>

#include "Controllers.hpp"
#include "DiagPIController.hpp"
#include "MIMOControlLoop.hpp"
#include "MeasurementT.hpp"
#include "NullLockin.hpp"
#include "PI_E727_Controller.hpp"
#include "SISOControlLoop.hpp"
#include "SPMCRingBuffer.hpp"
#include "SensorData.hpp"
#include "TSCircularBuffer.hpp"
#include "nF_EBD_Controller.hpp"
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

struct MetrologyResources {
  // actuators
  char serial_number1[1024] = "0122040101";
  char serial_number2[1024] = "0122042007";
  PI_E727_Controller tip_tilt_stage1{serial_number1};
  PI_E727_Controller tip_tilt_stage2{serial_number2};
  nF_EBD_Controller nF_stage_1{"/dev/ttyUSB2"};
  nF_EBD_Controller nF_stage_2{"/dev/ttyUSB3"};

  ShearXActuator shear_x1_actuator{tip_tilt_stage1};
  ShearXActuator shear_x2_actuator{tip_tilt_stage2};
  ShearYActuator shear_y1_actuator{tip_tilt_stage1};
  ShearYActuator shear_y2_actuator{tip_tilt_stage2};

  // control loop
  std::atomic<bool> reset_phase_unwrap = false;

  // data queues
  TSCircularBuffer<MeasurementT<int, int>> adc_queues[14];
  SPMCRingBuffer<SensorData, 100000> sensorDataQueue;

  // science beam
  NullLockin null_lockin;

  // controllers
  PIController shear_x1_controller;
  PIController shear_x2_controller;
  PIController shear_y1_controller;
  PIController shear_y2_controller;
  DiagPIController<2> point_1_controller;
  DiagPIController<2> point_2_controller;

  // control loops
  SISOControlLoop<PIController, ShearXActuator> shear_x1_loop{shear_x1_controller, shear_x1_actuator};
  SISOControlLoop<PIController, ShearXActuator> shear_x2_loop{shear_x2_controller, shear_x2_actuator};
  SISOControlLoop<PIController, ShearYActuator> shear_y1_loop{shear_y1_controller, shear_y1_actuator};
  SISOControlLoop<PIController, ShearYActuator> shear_y2_loop{shear_y2_controller, shear_y2_actuator};
  MIMOControlLoop<2, DiagPIController<2>, nF_EBD_Controller> point_1_loop{point_1_controller, nF_stage_1};
  MIMOControlLoop<2, DiagPIController<2>, nF_EBD_Controller> point_2_loop{point_2_controller, nF_stage_2};
};

struct SharedResources {
  MetrologyResources metrology;
};
