#pragma once

#include <cmath>

// Follows the difference of two clocks with a Kalman filter of two states.
//
//   offset  the DC clock minus the PC clock, in seconds
//   rate    how much faster the DC clock runs, without a unit (1e-6 is one microsecond in each second)
//
// The measurement is one pair of the two clocks. Between two pairs the offset moves by rate * dt, and the rate
// itself moves as a random walk with the power kRateRandomWalk.
//
// The constants come from records of the NICE bus: the noise of one pair is 100 ns, and the two clocks move against
// each other by some microseconds with a period of about 30 s. kRateRandomWalk gives a time constant of 0.71 s at a
// period of 10 ms, thus the filter follows that movement and keeps an uncertainty of 12 ns on the offset and 12 ppb
// on the rate.
//
// A pair that is further away than kGate deviations does not enter the filter, because a slow register read gives a
// pair that is some microseconds late. After kMaxRejected such pairs the two clocks made a real jump, thus the
// filter starts again with the newest pair.
class ClockFilter {
 public:
  static constexpr double kMeasurementNoise = 100e-9;  // s, the noise of one pair
  static constexpr double kRateRandomWalk = 1e-16;     // s^2/s^3
  static constexpr double kInitialRateSd = 100e-6;     // the rate is unknown at the start
  static constexpr double kGate = 5.0;
  static constexpr int kMaxRejected = 100;

  bool locked() const { return locked_; }
  double offset() const { return offset_; }             // s
  double rate() const { return rate_; }                 // without a unit
  double offset_sd() const { return std::sqrt(p00_); }  // s
  double rate_sd() const { return std::sqrt(p11_); }    // without a unit

  void reset() { locked_ = false; }

  // The offset at dt seconds after the last pair, without a new measurement.
  double offset_at(double dt) const { return offset_ + rate_ * dt; }

  // Take one pair. dt is the period since the last pair in seconds, measurement is the offset of the pair in
  // seconds. Gives false if the gate rejected the pair.
  bool update(double dt, double measurement) {
    if (!locked_) {
      start(measurement);
      return true;
    }

    // Predict. The state moves by rate * dt, and the covariance grows by the random walk of the rate.
    offset_ += rate_ * dt;
    p00_ += dt * (2.0 * p01_ + dt * p11_) + kRateRandomWalk * dt * dt * dt / 3.0;
    p01_ += dt * p11_ + kRateRandomWalk * dt * dt / 2.0;
    p11_ += kRateRandomWalk * dt;

    const double variance = p00_ + kMeasurementNoise * kMeasurementNoise;
    const double innovation = measurement - offset_;
    if (std::abs(innovation) > kGate * std::sqrt(variance)) {
      if (++rejected_ >= kMaxRejected) {
        start(measurement);
      }
      return false;
    }
    rejected_ = 0;

    // Update.
    const double gain_offset = p00_ / variance;
    const double gain_rate = p01_ / variance;
    const double p01_before = p01_;
    offset_ += gain_offset * innovation;
    rate_ += gain_rate * innovation;
    p00_ -= gain_offset * p00_;
    p01_ -= gain_offset * p01_;
    p11_ -= gain_rate * p01_before;
    return true;
  }

 private:
  bool locked_ = false;
  int rejected_ = 0;
  double offset_ = 0.0;
  double rate_ = 0.0;
  double p00_ = 0.0;
  double p01_ = 0.0;
  double p11_ = 0.0;

  // The first pair gives the offset. The rate is unknown, thus its variance is large and the next pairs find it.
  void start(double measurement) {
    locked_ = true;
    rejected_ = 0;
    offset_ = measurement;
    rate_ = 0.0;
    p00_ = kMeasurementNoise * kMeasurementNoise;
    p01_ = 0.0;
    p11_ = kInitialRateSd * kInitialRateSd;
  }
};
