#pragma once

#include <array>
#include <atomic>
#include <thread>

#include "Controllers.hpp"
#include "PI_E727_Controller.hpp"
#include "PiezoActuators.hpp"
#include "SharedResources.hpp"
#include "nF_EBD_Controller.hpp"

/**
 * @brief Calculates control loop commands based on sensor measurements, and sends them to the actuators.
 *
 * Open loop control: Send raw actuator commands
 * Closed-loop control: Use sensor measurements to derive an actuator command, with the goal of achieving a desired
 * setpoint. Control configuration: Setpints, PID gains etc.
 */

/*
 * Implementation overview:
 *
 * The control loops run in a separate thread, which is executed at every sensor measurement from the EtherCAT system.
 * The EtherCAT cycle thus acts as a clock and source of timestamps for the control loops. We follow a sense-plan-act
 * paradigm:
 * - Sense: Read sensor data from the EtherCAT system, and calculate the current state of the system. Load user-defined
 * setpoints etc.
 * - Plan: Calculate the desired actuator commands based on the current state and the control configuration.
 * - Act: Send the actuator commands to the actuators.
 *
 * The outside world interacts via functions like move_to_x1, start_loop_opd, set_shear_pi_gains etc. Atomics are used
 * to relay the commands to the control loop.
 * - S
 */

class ControlManager {
 public:
  ControlManager(SharedResources &res) : res(res), piezos(res.piezos) { setup(); }

  ~ControlManager() {
    piezos.tt1.close();
    piezos.tt2.close();
    // piezos.tt3.close();
    // piezos.tt4.close();
  }

  void start() {
    if (control_thread.joinable()) {
      return;
    }  // already running
    control_thread = std::jthread([this](std::stop_token st) {
      while (!st.stop_requested()) {
        control();
      }
    });
  }

  void request_stop() {
    if (control_thread.joinable()) {
      control_thread.request_stop();
    }
  }

  // beam control, coordinates axes are at the spatial filter collimator entrance
  // The minus sign is due to field inversion in the periscope
  void move_to_x1(float position) { shear_y1_cmd.store(-position); }
  void move_to_y1(float position) { shear_x1_cmd.store(-position); }
  void move_to_x2(float position) { shear_y2_cmd.store(position); }
  void move_to_y2(float position) { shear_x2_cmd.store(position); }

  void set_shear_loop_select(int select) { shear_loop_select.store(select); }

 private:
  std::jthread control_thread;
  SharedResources &res;
  PiezoActuators &piezos;

  // tt1 open loop setpoints as array
  std::atomic<float> shear_x1_cmd{0.0f};
  std::atomic<float> shear_y1_cmd{0.0f};
  std::atomic<float> shear_x2_cmd{0.0f};
  std::atomic<float> shear_y2_cmd{0.0f};

  std::atomic<int> shear_loop_select{0};  // 0: open loop, 1: closed loop

  void control() {
    // switch statement for shear control loop
    switch (shear_loop_select.load()) {
      case 0:  // raw actuator commands
        piezos.tt1.move_to_x(shear_x1_cmd.load());
        piezos.tt1.move_to_y(shear_y1_cmd.load());
        piezos.tt2.move_to_x(shear_x2_cmd.load());
        piezos.tt2.move_to_y(shear_y2_cmd.load());
        break;
      default:
        break;  // do nothing
    }
  }

  void setup() {
    // Tip/tilt stage 1
    piezos.tt1.init();
    // piezos.tt1.autozero(); // run autozero if stage does not move
    // piezos.tt1.move_to_x(0.0f);
    // piezos.tt1.move_to_y(0.0f);
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // std::cout << "\tPI Stage 1 Position: (" << piezos.tt1.readx() << ", " << piezos.tt1.ready() << ") urad"
    //           << std::endl;

    // Tip/tilt stage 2
    piezos.tt2.init();
    // piezos.tt2.autozero(); // run autozero if stage does not move
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
