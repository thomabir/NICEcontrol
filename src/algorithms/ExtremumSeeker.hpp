#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include "algorithms/Controllers.hpp"
#include "algorithms/IirFilter.hpp"
#include "data/ExtremumSeeker.hpp"

// Extremum seeking on one measurement and one plant input.
//
// Something adds a sine of the amplitude a to the input of the plant, and the measurement follows that sine. At an
// extremum the first harmonic of the measurement goes to zero, and it changes its sign there, thus it is a
// measurement of the gradient:
//
//   gradient = 2 * lowpass((measurement - mean) * sin(dither phase - demodulation phase) / a)
//
// A PI controller drives that gradient to zero, and its output moves the plant. The division by the amplitude keeps
// the amplitude out of the gain of the loop, thus the amplitude is free to change while the seeker runs. Each sample
// takes the amplitude that the plant had when that sample was taken, thus a new amplitude does not rescale what the
// low pass already holds. The division by the size of the mean, if the caller asks for it, keeps the size of the
// measurement out of the gain as well.
//
// The output only moves while the estimate of the gradient is one that the seeker can trust. It holds the output:
//   from the start of a run, and from any change of what the estimate is made of, until the low pass has settled
//   while the measurement is gone for longer than the timeout
//   while the dither is off
//
// A rate bounds how fast the output moves at all, thus a gradient without bound, as the division by a mean that goes
// through zero gives, cannot make the plant jump.
//
// The class knows no hardware, thus one seeker fits any pair of a measurement and a plant input. The caller gives
// each sample the phase that the dither had when that sample was taken, so a late sample carries its own phase and
// nothing has to be synchronised.
//
// One step of the seeker, at a fixed sample time:
//   begin_step(config, amplitude)  takes the settings and the amplitude of the dither, both free to change at any step
//   add_sample(value, phase)       any number of times, including none
//   step()                         makes the new output and reports the state
class ExtremumSeeker {
 public:
  explicit ExtremumSeeker(double sample_time_s) : ts(sample_time_s), pi(0.0, 0.0, sample_time_s) {
    set_lowpass(cfg.lowpass_tau_s);
    reset_estimate();
  }

  void begin_step(const ExtremumSeekerConfig &settings, double dither_amplitude) {
    // The gradient is made of these four. A new value of any of them makes the low pass hold an estimate of
    // something else, thus the output waits until that estimate is gone.
    const bool estimate_changed = dither_amplitude != amplitude || settings.lowpass_tau_s != cfg.lowpass_tau_s ||
                                  settings.demod_phase_rad != cfg.demod_phase_rad ||
                                  settings.normalise != cfg.normalise;
    cfg = settings;
    amplitude = dither_amplitude;
    set_lowpass(cfg.lowpass_tau_s);
    pi.setGains(cfg.kp, cfg.ki);
    if (estimate_changed) {
      hold_until_settled();
    }
    sum_product = 0.0;
    sum_value = 0.0;
    count = 0;
  }

  // The mean comes off the value before the demodulation. It carries nothing about the gradient, and it would
  // otherwise leave a large ripple at the frequency of the dither in the output of the low pass.
  // The first sample of all puts the low pass of the mean where it belongs, thus the seeker does not spend the
  // settling time of that filter with a gradient that is only the mean.
  void add_sample(double value, double dither_phase_rad) {
    if (state.stale) {
      reset_estimate();  // what the low pass holds is from before the gap in the measurement, thus it goes
    }
    if (!primed) {
      primed = true;
      state.mean = value;
      held_value = value;
      mean_filter.preset(value);
    }
    if (amplitude > 0.0) {
      sum_product +=
          (value - state.mean) * std::sin(dither_phase_rad - static_cast<double>(cfg.demod_phase_rad)) / amplitude;
    }
    sum_value += value;
    count++;
  }

  // A step without a sample holds the measurement of the step before, thus the controller keeps its sample time.
  const ExtremumSeekerState &step() {
    if (count > 0) {
      held_product = sum_product / count;
      held_value = sum_value / count;
      state.sample_count += static_cast<uint64_t>(count);
      steps_without_sample = 0;
    } else if (!state.stale) {
      steps_without_sample++;  // it stops at the timeout, thus the count of a long gap stays inside its type
    }
    state.stale = static_cast<double>(steps_without_sample) * ts > static_cast<double>(cfg.sample_timeout_s);
    state.mean = mean_filter.filter(held_value);
    state.gradient = gradient(product_filter.filter(held_product));
    if (settle_steps > 0) {
      settle_steps--;
    }
    state.settling = settle_steps > 0;
    if (!state.running) {
      return state;
    }

    // The output goes against the gradient towards a minimum, and with it towards a maximum. An error that pushes
    // the output further past a limit does not reach the controller, so that the integral does not wind up. An
    // estimate that the seeker cannot trust reaches it as no error at all, thus the output stays where it is.
    double error = 0.0;
    state.at_max_rate = false;
    if (!state.settling && !state.stale && amplitude > 0.0) {
      error = cfg.direction * state.gradient;
      if ((above && error > 0.0) || (below && error < 0.0)) {
        error = 0.0;
      }
      const double largest = largest_error();
      if (std::abs(error) > largest) {
        error = std::clamp(error, -largest, largest);
        state.at_max_rate = true;
      }
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
    reset_estimate();
    state.running = true;
    state.output = output;
    state.offset = 0.0;
    state.at_limit = false;
    state.sample_count = 0;
  }

  void stop() {
    state.running = false;
    state.offset = 0.0;
    state.at_limit = false;
  }

  // Throws the estimate of the gradient away, for a caller whose measurement is now another one. The output holds
  // until the new estimate has settled.
  void reset_estimate() {
    primed = false;
    product_filter.reset();
    mean_filter.reset();
    sum_product = 0.0;
    sum_value = 0.0;
    count = 0;
    held_product = 0.0;
    held_value = 0.0;
    steps_without_sample = 0;
    state.gradient = 0.0;
    state.stale = false;
    hold_until_settled();
  }

  bool running() const { return state.running; }

 private:
  // Only a guard against a division by zero. A mean that small carries no gradient either.
  static constexpr double kMinMean = 1e-12;

  // How many time constants of the low pass the output holds after the estimate of the gradient changes.
  static constexpr double kSettleTaus = 3.0;

  // The gain below which the integral moves the output by nothing, thus no rate holds it back.
  static constexpr double kMinKi = 1e-12;

  const double ts;  // the sample time of the controller and of the low pass
  PIController<double> pi;
  IirFilter<double, 2> product_filter{std::array<double, 2>{0.0, 0.0}, std::array<double, 1>{0.0}};
  IirFilter<double, 2> mean_filter{std::array<double, 2>{0.0, 0.0}, std::array<double, 1>{0.0}};
  double tau_now = 0.0;

  ExtremumSeekerConfig cfg;
  ExtremumSeekerState state;
  double amplitude = 0.0;  // of the dither of this step

  bool primed = false;
  double sum_product = 0.0;
  double sum_value = 0.0;
  int count = 0;
  double held_product = 0.0;  // of the newest step that carried a sample
  double held_value = 0.0;    // of the newest step that carried a sample
  int steps_without_sample = 0;
  int settle_steps = 0;

  double base = 0.0;  // the output where the seeker started
  bool above = false;
  bool below = false;

  // The error that moves the integral of the controller by the largest rate that the caller allows. The gradient
  // grows without bound when the measurement it is divided by goes to zero, and a plant that jumps is worse than one
  // that takes longer, thus the error and not the output carries the rate: the integral then holds no motion that
  // the seeker did not make.
  double largest_error() const {
    const double ki = std::abs(static_cast<double>(cfg.ki));
    if (ki < kMinKi) {
      return std::numeric_limits<double>::infinity();
    }
    return std::abs(static_cast<double>(cfg.max_rate)) / ki;
  }

  // The first harmonic of a sine of the amplitude a is a / 2 times the gradient. The samples already carry the
  // division by a.
  double gradient(double product_lp) const {
    if (!(amplitude > 0.0)) {
      return 0.0;
    }
    const double slope = 2.0 * product_lp;
    if (!cfg.normalise) {
      return slope;
    }
    // The size of the mean, and not the mean itself: a mean that goes through zero would otherwise turn the sign of
    // the gradient over, and the loop would run away from the extremum instead of towards it.
    const double size = std::abs(state.mean);
    if (size < kMinMean) {
      return 0.0;
    }
    return slope / size;
  }

  void hold_until_settled() {
    const double settle_s = kSettleTaus * std::max(static_cast<double>(cfg.lowpass_tau_s), ts);
    settle_steps = std::max(settle_steps, static_cast<int>(settle_s / ts) + 1);
    state.settling = true;
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
