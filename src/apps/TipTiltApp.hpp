#pragma once

#include <array>
#include <future>

#include "core/Commands.hpp"
#include "core/Whiteboard.hpp"
#include "devices/PI_E727_Controller.hpp"
#include "devices/nF_EBD_Controller.hpp"

// Lateral beam control with two PI tip/tilt stages and two nanoFaktur tip/tilt stages.
// The setpoints and the sensor values are in the local axes of each actuator. See data/TipTilt.hpp for the map to the
// beam coordinates. The reads of all actuators run in parallel, thus the sense step takes the time of the slowest read.
class TipTiltApp {
 public:
  explicit TipTiltApp(Whiteboard &whiteboard) : wb(whiteboard) {}

  // Connects to the actuators. This is slow, so the core calls it before the first cycle.
  void init() {
    TipTiltState &state = wb.state.tiptilt;
    state.connected = true;
    for_each_controller([&](int i, auto &controller) {
      TipTiltActuatorState &actuator = state.actuators[i];
      actuator.connected = controller.init();
      state.connected = state.connected && actuator.connected;
      actuator.min_local_mrad = controller.range_min();
      actuator.max_local_mrad = controller.range_max();
    });
  }

  void sense() {
    TipTiltState &state = wb.state.tiptilt;
    std::array<std::future<tiptilt::Vector2>, kTipTiltActuators> reads;
    for_each_controller([&](int i, auto &controller) {
      if (state.actuators[i].connected) {
        reads[i] = std::async(std::launch::async, [&controller] { return controller.read(); });
      }
    });
    for (int i = 0; i < kTipTiltActuators; i++) {
      if (reads[i].valid()) {
        state.actuators[i].sensor_local_mrad = reads[i].get();
      }
    }
  }

  void act(const TipTiltCommands &command) {
    if (command.mode != 0) {
      return;  // only the open loop mode sends commands
    }

    // The stages hold their position, so a command goes out only when the setpoint moves.
    for_each_controller([&](int i, auto &controller) {
      const TipTiltSetpoint &setpoint = command.setpoints[i];
      if (!wb.state.tiptilt.actuators[i].connected || !setpoint.set || setpoint == sent[i]) {
        return;
      }
      controller.move_to(setpoint.local_mrad);
      sent[i] = setpoint;
    });
  }

 private:
  Whiteboard &wb;
  PI_E727_Controller pi1{"0122040101"};
  PI_E727_Controller pi2{"0122042007"};
  nF_EBD_Controller nf1{"1000003878"};
  nF_EBD_Controller nf2{"1000003879"};
  std::array<TipTiltSetpoint, kTipTiltActuators> sent;

  // Calls f(i, controller) for each actuator, in the order of kTipTiltActuatorNames.
  template <typename F>
  void for_each_controller(F f) {
    static_assert(kTipTiltActuators == 4);
    f(0, pi1);
    f(1, pi2);
    f(2, nf1);
    f(3, nf2);
  }
};
