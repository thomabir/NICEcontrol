#pragma once

#include <cstdint>
#include <memory>

#include "algorithms/ClockFilter.hpp"
#include "core/ClockPublisher.hpp"
#include "core/Whiteboard.hpp"

namespace ecat {
class DcClock;
}

/**
 * @brief Gives the EtherCAT distributed clock (DC) to every part of the program.
 *
 * The card gives one pair of the two clocks each millisecond, and the core reads the newest pair each cycle.
 * Each pair enters a Kalman filter of two states, and the offset of the two clocks goes to wb.clocks.
 * After that, each cycle writes the two clocks into shared memory, for the other programs on this PC.
 *
 * The program runs without the card: the constructor then reports the reason one time and the application stays inactive. The esd stack permits one open card in each process, thus a second attempt cannot succeed.
 */
class ClockApp {
 public:
  explicit ClockApp(Whiteboard &whiteboard);
  ~ClockApp();

  void sense();

 private:
  Whiteboard &wb;
  std::unique_ptr<ecat::DcClock> clock;
  ClockPublisher publisher;
  ClockFilter filter;
  int64_t last_dc_ns = 0;
  int64_t last_pc_ns = 0;

  void read_card();

  // The two clocks for the other programs on this PC, and the health of the offset for the user interface.
  // A record appears only while the offset is good, thus a fresh record is a good record.
  void publish();

  // Put one pair of t_DC and t_PC into the filter and give the offset of the two clocks to the whiteboard.
  void take(int64_t dc_ns, int64_t pc_ns);
};
