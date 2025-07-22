#pragma once

#include "PI_E727_Controller.hpp"
#include "PiezoActuators.hpp"
#include "nF_EBD_Controller.hpp"

/**
 * @brief Calculates the control loop and commands the tip/tilt piezo stages for control of pointing and shear.
 *
 * The functions for open loop control send commands directly to the actuators.
 * TODO The functions for closed loop control use measurements from the laser metrology to achieve a desired setpoint
 * with a PI controller.
 *
 */

class BeamController {
 public:
  BeamController(PiezoActuators &piezos) : piezos(piezos) { initialise_stages(); }
  ~BeamController() {
    piezos.tt1.close();
    piezos.tt2.close();
    // piezos.tt3.close();
    // piezos.tt4.close();
  }

  void start() {}

  void request_stop() {}

  // beam control, coordinates at the spatial filter collimator entrance
  void move_to_x1(float position) { piezos.tt1.move_to_x(position); }
  void move_to_y1(float position) { piezos.tt1.move_to_y(position); }
  void move_to_x2(float position) { piezos.tt2.move_to_x(position); }
  void move_to_y2(float position) { piezos.tt2.move_to_y(position); }

 private:
  PiezoActuators &piezos;

  void initialise_stages() {
    // Tip/tilt stage 1
    piezos.tt1.init();
    // tt1.autozero(); // run autozero if stage does not move
    // piezos.tt1.move_to_x(0.0f);
    // piezos.tt1.move_to_y(0.0f);
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // std::cout << "\tPI Stage 1 Position: (" << piezos.tt1.readx() << ", " << piezos.tt1.ready() << ") urad"
    //           << std::endl;

    // Tip/tilt stage 2
    piezos.tt2.init();
    // tt2.autozero(); // run autozero if stage does not move
    // piezos.tt2.move_to_x(0.0f);
    // piezos.tt2.move_to_y(0.0f);
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // std::cout << "\tPI Stage 2 Position: (" << piezos.tt2.readx() << ", " << piezos.tt2.ready() << ") urad"
    //           << std::endl;

    // nF tip/tilt stages
    // tt3.init();
    // tt3.move_to({0.0, 0.0});
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // auto pos = tt3.read();
    // std::cout << "nF Stage 1 Position: " << pos[0] << ", " << pos[1] << std::endl;

    // tt4.init();
    // tt4.move_to({0.0, 0.0});
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // pos = tt4.read();
    // std::cout << "nF Stage 2 Position: " << pos[0] << ", " << pos[1] << std::endl;
  }
};
