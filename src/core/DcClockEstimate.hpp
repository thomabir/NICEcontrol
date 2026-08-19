#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <optional>

// The DC time of any PC time, for every part of the program.
//
//   if (auto t = wb.dc_clock.t_DC_now())        // or t_DC_from_t_PC(some monotonic PC time)
//       use(t->dc_ns);                          // nanoseconds from 2000-01-01 00:00
//
// ClockApp measures the two clocks and publishes the result here after each core cycle. Any thread reads it at any
// time. The value is empty while the card gives no clock.
//
// The estimate is a straight line: the DC time at the reference, and the rate of the DC clock against the PC clock.
// ClockApp puts the reference on the newest pair, thus a caller of t_DC_now() extrapolates over one core period.
class DcClockEstimate {
 public:
  struct Estimate {
    int64_t dc_ns = 0;      // the DC time, from 2000-01-01 00:00
    double sigma_ns = 0.0;  // the uncertainty of dc_ns
    double rate = 0.0;      // how much faster the DC clock runs, without a unit
  };

  // ClockApp calls this after each new pair. reference is the PC time of the pair, dc_ns its DC time.
  void publish(std::chrono::steady_clock::time_point reference, int64_t dc_ns, double rate, double sigma_ns,
               double rate_sd) {
    std::lock_guard<std::mutex> lock(mutex_);
    valid_ = true;
    reference_ = reference;
    dc_ns_ = dc_ns;
    rate_ = rate;
    sigma_ns_ = sigma_ns;
    rate_sd_ = rate_sd;
  }

  // The card gives no clock, thus no part of the program gets a DC time.
  void invalidate() {
    std::lock_guard<std::mutex> lock(mutex_);
    valid_ = false;
  }

  [[nodiscard]] std::optional<Estimate> t_DC_from_t_PC(std::chrono::steady_clock::time_point t_PC) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!valid_) {
      return std::nullopt;
    }
    const double elapsed_ns =
        static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(t_PC - reference_).count());
    // The uncertainty of the rate adds to the uncertainty of the offset. The two are not independent, thus the sum
    // is an upper limit and not the exact deviation.
    return Estimate{dc_ns_ + static_cast<int64_t>(std::llround(elapsed_ns * (1.0 + rate_))),
                    sigma_ns_ + rate_sd_ * std::abs(elapsed_ns), rate_};
  }

  [[nodiscard]] std::optional<Estimate> t_DC_now() const { return t_DC_from_t_PC(std::chrono::steady_clock::now()); }

 private:
  mutable std::mutex mutex_;
  bool valid_ = false;
  std::chrono::steady_clock::time_point reference_;
  int64_t dc_ns_ = 0;
  double rate_ = 0.0;
  double sigma_ns_ = 0.0;
  double rate_sd_ = 0.0;
};
