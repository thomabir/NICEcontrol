#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include "algorithms/Controllers.hpp"
#include "algorithms/IirFilter.hpp"
#include "data/ExtremumSeeker.hpp"

// Extremum seeking on one measurement and one plant input.
//
// Something adds a sine of the amplitude a to the input of the plant, and the measurement follows that sine. At an
// extremum the first harmonic of the measurement goes to zero, and it changes its sign there, thus it is a
// measurement of the gradient:
//
//   gradient = 2 / a * lowpass((measurement - mean) * sin(dither phase - demodulation phase))
//
// A PI controller drives that gradient to zero, and its output moves the plant. The division by the amplitude keeps
// the amplitude out of the gain of the loop, thus the amplitude is free to change while the seeker runs. The
// division by the mean, if the caller asks for it, keeps the size of the measurement out of the gain as well.
//
// The class knows no hardware, thus one seeker fits any pair of a measurement and a plant input. The caller gives
// each sample the phase that the dither had when that sample was taken, so a late sample carries its own phase and
// nothing has to be synchronised.
//
// One step of the seeker, at a fixed sample time:
//   begin_step(config)        takes the settings, which are free to change at any step
//   add_sample(value, phase)  any number of times, including none
//   step(amplitude)           makes the new output and reports the state
class ExtremumSeeker {
 public:
  explicit ExtremumSeeker(double sample_time_s) : ts(sample_time_s), pi(0.0, 0.0, sample_time_s) {
    set_lowpass(ExtremumSeekerConfig{}.lowpass_tau_s);
  }

  void begin_step(const ExtremumSeekerConfig &settings) {
    cfg = settings;
    set_lowpass(cfg.lowpass_tau_s);
    pi.setGains(cfg.kp, cfg.ki);
    sum_product = 0.0;
    sum_value = 0.0;
    count = 0;
  }

  // The mean comes off the value before the demodulation. It carries nothing about the gradient, and it would
  // otherwise leave a large ripple at the frequency of the dither in the output of the low pass.
  // The first sample of all puts the low pass of the mean where it belongs, thus the seeker does not spend the
  // settling time of that filter with a gradient that is only the mean.
  void add_sample(double value, double dither_phase_rad) {
    if (!primed) {
      primed = true;
      state.mean = value;
      held_value = value;
      mean_filter.preset(value);
    }
    sum_product += (value - state.mean) * std::sin(dither_phase_rad - static_cast<double>(cfg.demod_phase_rad));
    sum_value += value;
    count++;
  }

  // A step without a sample holds the measurement of the step before, thus the controller keeps its sample time.
  const ExtremumSeekerState &step(double dither_amplitude) {
    if (count > 0) {
      held_product = sum_product / count;
      held_value = sum_value / count;
      state.sample_count += static_cast<uint64_t>(count);
    }
    state.mean = mean_filter.filter(held_value);
    state.gradient = gradient(product_filter.filter(held_product), dither_amplitude);
    if (!state.running) {
      return state;
    }

    // The output goes against the gradient towards a minimum, and with it towards a maximum. An error that pushes
    // the output further past a limit does not reach the controller, so that the integral does not wind up.
    double error = cfg.direction * state.gradient;
    if ((above && error > 0.0) || (below && error < 0.0)) {
      error = 0.0;
    }
    const double raw = pi.update(error);
    const double limit = std::abs(static_cast<double>(cfg.limit));
    above = raw > limit;
    below = raw < -limit;
    state.offset = std::clamp(raw, -limit, limit);
    state.output = base + state.offset;
    state.at_limit = above || below;
    return state;
  }

  // The seeker starts where the plant is now, and the controller starts at zero, thus the output does not jump.
  void start(double output) {
    base = output;
    pi.reset_Iir_filter();
    above = false;
    below = false;
    state.running = true;
    state.output = output;
    state.offset = 0.0;
    state.at_limit = false;
  }

  void stop() {
    state.running = false;
    state.offset = 0.0;
    state.at_limit = false;
  }

  bool running() const { return state.running; }

 private:
  // Only a guard against a division by zero. A mean that small carries no gradient either.
  static constexpr double kMinMean = 1e-12;

  const double ts;  // the sample time of the controller and of the low pass
  PIController<double> pi;
  IirFilter<double, 2> product_filter{std::array<double, 2>{0.0, 0.0}, std::array<double, 1>{0.0}};
  IirFilter<double, 2> mean_filter{std::array<double, 2>{0.0, 0.0}, std::array<double, 1>{0.0}};
  double tau_now = 0.0;

  ExtremumSeekerConfig cfg;
  ExtremumSeekerState state;

  bool primed = false;
  double sum_product = 0.0;
  double sum_value = 0.0;
  int count = 0;
  double held_product = 0.0;  // of the newest step that carried a sample
  double held_value = 0.0;    // of the newest step that carried a sample

  double base = 0.0;  // the output where the seeker started
  bool above = false;
  bool below = false;

  // The first harmonic of a sine of the amplitude a is a / 2 times the gradient.
  double gradient(double product_lp, double dither_amplitude) const {
    if (!(dither_amplitude > 0.0)) {
      return 0.0;
    }
    const double slope = 2.0 * product_lp / dither_amplitude;
    if (!cfg.normalise) {
      return slope;
    }
    if (std::abs(state.mean) < kMinMean) {
      return 0.0;
    }
    return slope / state.mean;
  }

  // A first order low pass, from the bilinear transform of 1 / (1 + s tau).
  void set_lowpass(float tau_s) {
    const double tau = std::max(static_cast<double>(tau_s), ts);
    if (tau == tau_now) {
      return;
    }
    tau_now = tau;
    const double k = 0.5 * ts / tau;
    const std::array<double, 2> b{k / (1.0 + k), k / (1.0 + k)};
    const std::array<double, 1> a{(1.0 - k) / (1.0 + k)};
    product_filter.set_coefficients(b, a);
    mean_filter.set_coefficients(b, a);
  }
};
