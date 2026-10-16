#pragma once

#include <array>

#include "core/Commands.hpp"
#include "core/Whiteboard.hpp"
#include "devices/PI_E727_Controller.hpp"
#include "devices/nF_EBD_Controller.hpp"

// Lateral beam control with two PI tip/tilt stages and two nanoFaktur tip/tilt stages.
// The setpoints and the sensor values are in the local axes of each actuator. See data/TipTilt.hpp for the map to the
// beam coordinates. The reads of all actuators run in parallel, thus the sense step takes the time of the slowest read.
class TipTiltApp {
 public:
  explicit TipTiltApp(Whiteboard &whiteboard);

  // Connects to the actuators. This is slow, so the core calls it before the first cycle.
  void init();

  void sense();
  void act(const TipTiltCommands &command);

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
