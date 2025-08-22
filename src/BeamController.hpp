#pragma once

#include <array>
#include <atomic>
#include <thread>

#include "Consumer.hpp"
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

  // beam control, coordinates at the spatial filter collimator entrance
  void move_to_x1(float position) { shear_y1_cmd.store(position); }
  void move_to_y1(float position) { shear_x1_cmd.store(-position); }
  void move_to_x2(float position) { shear_y2_cmd.store(-position); }
  void move_to_y2(float position) { shear_x2_cmd.store(position); }

  void set_shear_setpoints(float shear_x1, float shear_y1, float shear_x2, float shear_y2) {
    shear_x1_sp.store(shear_x1);
    shear_y1_sp.store(shear_y1);
    shear_x2_sp.store(shear_x2);
    shear_y2_sp.store(shear_y2);
  }
  void set_shear_gains(float p, float i) {
    shear_p.store(p);
    shear_i.store(i);
  }

  void set_shear_loop_select(int select) { shear_loop_select.store(select); }

 private:
  std::jthread control_thread;
  SharedResources &res;
  PiezoActuators &piezos;

  // sensor data
  EthercatData data;

  // tt1 open loop setpoints as array
  std::atomic<float> shear_x1_cmd{0.0f};
  std::atomic<float> shear_y1_cmd{0.0f};
  std::atomic<float> shear_x2_cmd{0.0f};
  std::atomic<float> shear_y2_cmd{0.0f};

  // control loop parameters
  std::atomic<float> shear_x1_sp{0.0f};
  std::atomic<float> shear_y1_sp{0.0f};
  std::atomic<float> shear_x2_sp{0.0f};
  std::atomic<float> shear_y2_sp{0.0f};
  std::atomic<float> shear_p{0.0f};
  std::atomic<float> shear_i{0.0f};
  std::atomic<int> shear_loop_select{0};  // 0: open loop, 1: closed loop

  void control() {
    static auto consumer = res.ethercat.data.subscribe();  // Subscribe to the EtherCAT data queue
    static PIController shear_x1_pi, shear_y1_pi, shear_x2_pi, shear_y2_pi;

    res.ethercat.data.try_pop(consumer, data);  // block until new data is available
    static float shear_p_gain = shear_p.load();
    static float shear_i_gain = shear_i.load();

    static float shear_x1_setpoint = shear_x1_sp.load();
    static float shear_y1_setpoint = shear_y1_sp.load();
    static float shear_x2_setpoint = shear_x2_sp.load();
    static float shear_y2_setpoint = shear_y2_sp.load();

    // calculate controller command
    static float shear_x1_command = shear_x1_pi.step(shear_x1_setpoint - data.metr_qpd[0]);  // QPD x1 channel
    static float shear_y1_command = shear_y1_pi.step(shear_y1_setpoint - data.metr_qpd[1]);  // QPD y1 channel
    static float shear_x2_command = shear_x2_pi.step(shear_x2_setpoint - data.metr_qpd[3]);  // QPD x2 channel
    static float shear_y2_command = shear_y2_pi.step(shear_y2_setpoint - data.metr_qpd[4]);  // QPD y2 channel

    // switch statement for shear control loop
    switch (shear_loop_select.load()) {
      case 0:  // raw actuator commands
        piezos.tt1.move_to_x(shear_x1_cmd.load());
        piezos.tt1.move_to_y(shear_y1_cmd.load());
        piezos.tt2.move_to_x(shear_x2_cmd.load());
        piezos.tt2.move_to_y(shear_y2_cmd.load());
        break;
      case 1:
        // read shear gains
        shear_p_gain = shear_p.load();
        shear_i_gain = shear_i.load();
        shear_x1_pi.setPI(shear_p_gain, shear_i_gain);
        shear_y1_pi.setPI(shear_p_gain, shear_i_gain);
        shear_x2_pi.setPI(shear_p_gain, shear_i_gain);
        shear_y2_pi.setPI(shear_p_gain, shear_i_gain);

        // read shear setpoints
        shear_x1_setpoint = shear_x1_sp.load();
        shear_y1_setpoint = shear_y1_sp.load();
        shear_x2_setpoint = shear_x2_sp.load();
        shear_y2_setpoint = shear_y2_sp.load();

        // calculate controller command
        shear_x1_command = shear_x1_pi.step(shear_x1_setpoint - data.metr_qpd[0]);  // QPD x1 channel
        shear_y1_command = shear_y1_pi.step(shear_y1_setpoint - data.metr_qpd[1]);  // QPD y1 channel
        shear_x2_command = shear_x2_pi.step(shear_x2_setpoint - data.metr_qpd[3]);  // QPD x2 channel
        shear_y2_command = shear_y2_pi.step(shear_y2_setpoint - data.metr_qpd[4]);  // QPD y2 channel

        // send command to actuator
        piezos.tt1.move_to_y(shear_x1_command);
        piezos.tt1.move_to_x(-shear_y1_command);
        piezos.tt2.move_to_y(-shear_x2_command);
        piezos.tt2.move_to_x(shear_y2_command);
        break;
      case 2:
        break;
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
