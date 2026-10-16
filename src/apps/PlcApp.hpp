#pragma once

#include <cstdint>
#include <memory>

#include "core/Commands.hpp"
#include "core/Whiteboard.hpp"
#include "data/PlcSample.hpp"

class PlcConnection;

// Reads the metrology and delay line samples from the PLC, and sends the optical path difference settings to it.
// The control loop itself runs on the PLC. This application only relays the settings.
class PlcApp {
 public:
  explicit PlcApp(Whiteboard &whiteboard);
  ~PlcApp();

  void sense();
  void act(const OpdCommands &command);

 private:
  Whiteboard &wb;
  std::unique_ptr<PlcConnection> plc;

  int retry_countdown = 1;
  uint32_t last_sample_no = 0;
  bool have_last_sample = false;

  OpdCommands sent;
  bool sent_valid = false;

  // A connection attempt is slow when the PLC is absent, so it happens once per retry interval and not once per cycle.
  static constexpr int kRetryCycles = 100;

  bool connect();
  void drop_connection();

  // The PLC is a device of the bus, thus its own timestamp is the distributed clock and the PC time follows from it.
  void take(const PlcSample &sample);
};
