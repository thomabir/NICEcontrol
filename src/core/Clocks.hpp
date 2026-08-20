#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>

#include "data/Timestamp.hpp"

// The two clocks of the program, in nanoseconds from 2000-01-01 00:00.
// t_PC is the monotonic clock of the PC, moved to that epoch one time at the start.
// t_DC is the distributed clock of the EtherCAT bus.
// ClockApp measures the offset of the two and writes it after each cycle.
class Clocks {
 public:
  // The t_PC of one reading of the monotonic clock, from the EtherCAT card or from a device server on this PC.
  int64_t t_PC_from_monotonic(std::chrono::steady_clock::time_point mono) const {
    return ns(mono.time_since_epoch()) + mono_to_PC_ns;
  }

  int64_t t_PC_now() const { return t_PC_from_monotonic(std::chrono::steady_clock::now()); }

  Timestamp stamp_now() const { return stamp_from_t_PC(t_PC_now()); }

  Timestamp stamp_from_monotonic(std::chrono::steady_clock::time_point mono) const {
    return stamp_from_t_PC(t_PC_from_monotonic(mono));
  }

  Timestamp stamp_from_t_DC(int64_t t_DC) const { return {t_DC - PC_to_DC_ns, t_DC}; }

  void set_offset(int64_t offset) { PC_to_DC_ns = offset; }

 private:
  static constexpr std::chrono::sys_days kEpoch{std::chrono::year{2000} / std::chrono::January / 1};

  template <typename Duration>
  static int64_t ns(Duration duration) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
  }

  Timestamp stamp_from_t_PC(int64_t t_PC) const { return {t_PC, t_PC + PC_to_DC_ns}; }

  // The wall clock gives the epoch, the monotonic clock gives the steps.
  // One reading of each at the start joins them, thus t_PC never goes backwards.
  static int64_t measure_mono_to_PC() {
    const int64_t mono = ns(std::chrono::steady_clock::now().time_since_epoch());
    return ns(std::chrono::system_clock::now() - kEpoch) - mono;
  }

  const int64_t mono_to_PC_ns = measure_mono_to_PC();
  std::atomic<int64_t> PC_to_DC_ns{0};
};
