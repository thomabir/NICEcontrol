#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>

#include "data/Timestamp.hpp"

/**
 * @brief The two clocks of the program, in nanoseconds from 2000-01-01 00:00.
 *
 * t_PC is the monotonic clock of the PC, moved to that epoch one time at the start, and t_DC is the distributed clock of the EtherCAT bus.
 * ClockApp measures the offset of the two and writes it after each cycle.
 *
 * The offset of the last estimate moves with the rate of that estimate, thus t_DC stays inside some microseconds of the bus while the pairs stop.
 * That holdover is good for kGoodAgeNs, and after that every timestamp says dc_good false.
 * Without any estimate t_DC is a copy of t_PC, thus the plots and the dither phase run without the card.
 */
class Clocks {
 public:
  /** The rate of the last estimate is correct to about 0.1 ppm, thus the holdover keeps 100 us for this period. */
  static constexpr int64_t kGoodAgeNs = 60'000'000'000;

  /** @brief The offset of the two clocks at one time of the PC clock. */
  struct Offset {
    bool good = false;       /**< the estimate exists and its last pair is younger than kGoodAgeNs */
    int64_t PC_to_DC_ns = 0; /**< t_DC = t_PC + PC_to_DC_ns */
  };

  /** @brief The t_PC of one reading of the monotonic clock, from the EtherCAT card or from a device server. */
  int64_t t_PC_from_monotonic(std::chrono::steady_clock::time_point mono) const {
    return ns(mono.time_since_epoch()) + mono_to_PC_ns;
  }

  int64_t t_PC_now() const { return t_PC_from_monotonic(std::chrono::steady_clock::now()); }

  /** @brief The step from the monotonic clock of this PC to t_PC, for the record of the other programs. */
  int64_t mono_to_PC() const { return mono_to_PC_ns; }

  Timestamp stamp_now() const { return stamp_from_t_PC(t_PC_now()); }

  Timestamp stamp_from_monotonic(std::chrono::steady_clock::time_point mono) const {
    return stamp_from_t_PC(t_PC_from_monotonic(mono));
  }

  Timestamp stamp_from_t_PC(int64_t t_PC) const {
    const Offset offset = offset_at_t_PC(t_PC);
    return {t_PC, t_PC + offset.PC_to_DC_ns, offset.good};
  }

  /**
   * @brief The timestamp of one time of the bus.
   *
   * The offset is a function of t_PC, and this caller knows t_DC only, thus the offset comes from the present time.
   * A sample from the bus is at most some milliseconds old, and the rate over that period is less than a nanosecond.
   */
  Timestamp stamp_from_t_DC(int64_t t_DC) const {
    const Offset offset = offset_at_t_PC(t_PC_now());
    return {t_DC - offset.PC_to_DC_ns, t_DC, offset.good};
  }

  /** @brief The offset of the two clocks at one time of the PC clock, from the last estimate and its rate. */
  Offset offset_at_t_PC(int64_t t_PC) const {
    std::lock_guard<std::mutex> lock(mutex);
    if (!has_estimate) {
      return {};
    }
    const int64_t dt = t_PC - estimate_t_PC;
    return {dt <= kGoodAgeNs, estimate_ns + std::llround(estimate_rate * static_cast<double>(dt))};
  }

  /**
   * @brief Takes one estimate of the filter.
   * @param PC_to_DC_ns the offset of the two clocks at t_PC_at
   * @param rate        how much faster the DC clock runs, without a unit
   * @param t_PC_at     the t_PC of the pair that gave the estimate
   */
  void set_offset(int64_t PC_to_DC_ns, double rate, int64_t t_PC_at) {
    std::lock_guard<std::mutex> lock(mutex);
    estimate_ns = PC_to_DC_ns;
    estimate_rate = rate;
    estimate_t_PC = t_PC_at;
    has_estimate = true;
  }

 private:
  static constexpr std::chrono::sys_days kEpoch{std::chrono::year{2000} / std::chrono::January / 1};

  template <typename Duration>
  static int64_t ns(Duration duration) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
  }

  // The wall clock gives the epoch, the monotonic clock gives the steps.
  // One reading of each at the start joins them, thus t_PC never goes backwards.
  static int64_t measure_mono_to_PC() {
    const int64_t mono = ns(std::chrono::steady_clock::now().time_since_epoch());
    return ns(std::chrono::system_clock::now() - kEpoch) - mono;
  }

  const int64_t mono_to_PC_ns = measure_mono_to_PC();

  // The last estimate of the filter. mutex guards the four members, because one reader must see one estimate.
  mutable std::mutex mutex;
  bool has_estimate = false;
  int64_t estimate_ns = 0;
  double estimate_rate = 0.0;
  int64_t estimate_t_PC = 0;
};
