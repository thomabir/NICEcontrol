#pragma once

#include "../Commands.hpp"
#include "../PI_E727_Controller.hpp"
#include "../Whiteboard.hpp"

// Lateral beam control with the two PI tip/tilt stages.
// A read of one encoder axis takes a USB round trip, so each cycle reads one axis and the four axes take turns.
class TipTiltApp {
 public:
  explicit TipTiltApp(Whiteboard &whiteboard) : wb(whiteboard) {}

  // Connects to the stages. This is slow, so the core calls it before the first cycle.
  void init() {
    const bool stage_1 = tt1.init();
    const bool stage_2 = tt2.init();
    wb.state.tiptilt.connected = stage_1 && stage_2;
  }

  void sense() {
    if (!wb.state.tiptilt.connected) {
      return;
    }

    TipTiltState &state = wb.state.tiptilt;
    switch (axis_turn) {
      case 0:
        state.x1 = tt1.readx();
        break;
      case 1:
        state.y1 = tt1.ready();
        break;
      case 2:
        state.x2 = tt2.readx();
        break;
      default:
        state.y2 = tt2.ready();
        break;
    }
    axis_turn = (axis_turn + 1) % 4;
  }

  void act(const TipTiltCommands &command) {
    if (!wb.state.tiptilt.connected || command.mode != 0) {
      return;  // only the open loop mode sends commands
    }

    // The stages hold their position, so a command goes out only when the target moves.
    if (sent_valid && command.x1 == sent.x1 && command.y1 == sent.y1 && command.x2 == sent.x2 &&
        command.y2 == sent.y2) {
      return;
    }

    // The coordinates are at the entrance of the spatial filter collimator. The minus signs and the exchange of the
    // axes are due to the field inversion in the periscope.
    tt1.move_to_y(-command.x1);
    tt1.move_to_x(-command.y1);
    tt2.move_to_y(command.x2);
    tt2.move_to_x(command.y2);

    sent = command;
    sent_valid = true;
  }

 private:
  Whiteboard &wb;
  char serial_number_1[1024] = "0122040101";
  char serial_number_2[1024] = "0122042007";
  PI_E727_Controller tt1{serial_number_1};
  PI_E727_Controller tt2{serial_number_2};
  int axis_turn = 0;
  TipTiltCommands sent;
  bool sent_valid = false;
};
