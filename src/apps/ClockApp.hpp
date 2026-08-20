#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>

#include "algorithms/ClockFilter.hpp"
#include "core/Whiteboard.hpp"
#include "devices/EthercatDcClock.hpp"

// Gives the EtherCAT distributed clock (DC) to every part of the program.
// The card gives one pair of the two clocks each millisecond, and the core reads the newest pair each cycle.
// Each pair enters a Kalman filter of two states, and the offset of the two clocks goes to wb.clocks.
// The program runs without the card.
// The constructor then reports the reason one time and the application stays inactive.
// The esd stack permits one open card in each process, thus a second attempt cannot succeed.
class ClockApp {
 public:
  explicit ClockApp(Whiteboard &whiteboard) : wb(whiteboard) {
    try {
      clock = std::make_unique<ecat::DcClock>();
    } catch (const std::exception &error) {
      std::cerr << "ClockApp: the DC clock is not available.\n" << error.what() << std::endl;
      return;
    }
    wb.state.clock.card_open = true;
  }

  void sense() {
    if (!clock) {
      return;
    }

    ClockState &state = wb.state.clock;
    state.al_state = clock->state();

    const std::optional<ecat::DcSample> sample = clock->get_DC_sample();
    state.clock_present = sample.has_value();
    if (!sample) {
      filter.reset();
      state.locked = false;
      return;
    }
    if (sample->dc_ns == last_dc_ns) {
      return;  // the card has no new sample since the last cycle
    }
    last_dc_ns = sample->dc_ns;

    const int64_t pc_ns = wb.clocks.t_PC_from_monotonic(sample->pc);
    state.dc_ns = sample->dc_ns;
    state.read_span_us = 1e-3 * static_cast<double>(sample->read_span.count());
    state.age_ms = 1e-6 * static_cast<double>(wb.clocks.t_PC_now() - pc_ns);
    state.sample_count++;

    take(sample->dc_ns, pc_ns);
  }

 private:
  Whiteboard &wb;
  std::unique_ptr<ecat::DcClock> clock;
  ClockFilter filter;
  int64_t last_dc_ns = 0;
  int64_t last_pc_ns = 0;

  // Put one pair of t_DC and t_PC into the filter and give the offset of the two clocks to the whiteboard.
  void take(int64_t dc_ns, int64_t pc_ns) {
    if (!filter.locked()) {
      last_pc_ns = pc_ns;
    }

    // The offset of this pair, and the estimate of that offset before this pair enters the filter.
    const double offset = 1e-9 * static_cast<double>(dc_ns - pc_ns);
    const double dt = 1e-9 * static_cast<double>(pc_ns - last_pc_ns);
    const double predicted_offset = filter.locked() ? filter.offset_at(dt) : offset;
    last_pc_ns = pc_ns;

    ClockState &state = wb.state.clock;
    if (!filter.update(dt, offset)) {
      state.rejected_count++;
    }
    state.locked = filter.locked();
    state.rate_ppm = filter.rate() * 1e6;
    state.offset_sd_ns = filter.offset_sd() * 1e9;
    state.rate_sd_ppb = filter.rate_sd() * 1e9;
    state.error_ns = (predicted_offset - offset) * 1e9;

    wb.clocks.set_offset(std::llround(filter.offset() * 1e9));
  }
};
