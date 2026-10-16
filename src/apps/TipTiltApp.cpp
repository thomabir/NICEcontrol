#include "apps/TipTiltApp.hpp"

#include <future>

TipTiltApp::TipTiltApp(Whiteboard &whiteboard) : wb(whiteboard) {}

void TipTiltApp::init() {
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

void TipTiltApp::sense() {
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

void TipTiltApp::act(const TipTiltCommands &command) {
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
