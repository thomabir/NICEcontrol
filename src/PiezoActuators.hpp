#pragma once

#include "PI_E727_Controller.hpp"
#include "nF_EBD_Controller.hpp"
// #include "Controllers.hpp"
// #include "DiagPIController.hpp"
// #include "MIMOControlLoop.hpp"
// #include "SISOControlLoop.hpp"

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

struct PiezoActuators {
  char serial_number1[1024] = "0122040101";
  char serial_number2[1024] = "0122042007";
  PI_E727_Controller tt1{serial_number1};
  PI_E727_Controller tt2{serial_number2};
  nF_EBD_Controller tt3{"/dev/ttyUSB2"};
  nF_EBD_Controller tt4{"/dev/ttyUSB3"};

  // ShearXActuator shear_x1_actuator{tt1};
  // ShearXActuator shear_x2_actuator{tt2};
  // ShearYActuator shear_y1_actuator{tt1};
  // ShearYActuator shear_y2_actuator{tt2};

  // controllers
  // PIController shear_x1_controller;
  // PIController shear_x2_controller;
  // PIController shear_y1_controller;
  // PIController shear_y2_controller;
  // DiagPIController<2> point_1_controller;
  // DiagPIController<2> point_2_controller;

  // control loops
  // SISOControlLoop<PIController, ShearXActuator> shear_x1_loop{shear_x1_controller, shear_x1_actuator};
  // SISOControlLoop<PIController, ShearXActuator> shear_x2_loop{shear_x2_controller, shear_x2_actuator};
  // SISOControlLoop<PIController, ShearYActuator> shear_y1_loop{shear_y1_controller, shear_y1_actuator};
  // SISOControlLoop<PIController, ShearYActuator> shear_y2_loop{shear_y2_controller, shear_y2_actuator};
  // MIMOControlLoop<2, DiagPIController<2>, nF_EBD_Controller> point_1_loop{point_1_controller, tt3};
  // MIMOControlLoop<2, DiagPIController<2>, nF_EBD_Controller> point_2_loop{point_2_controller, tt4};
};
