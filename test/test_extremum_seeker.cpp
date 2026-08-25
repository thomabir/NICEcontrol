// Checks the dither phase in src/algorithms/Dither.hpp and the seeker in src/algorithms/ExtremumSeeker.hpp.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <numbers>
#include <string>

#include "algorithms/Dither.hpp"
#include "algorithms/ExtremumSeeker.hpp"

static int failures = 0;

static void expect_near(const std::string &what, double got, double want, double tolerance) {
  const bool ok = std::abs(got - want) <= tolerance;
  std::printf("%-52s got %+.6f want %+.6f +- %.6f  %s\n", what.c_str(), got, want, tolerance, ok ? "OK" : "FAIL");
  if (!ok) failures++;
}

static void expect_below(const std::string &what, double got, double bound) {
  const bool ok = got <= bound;
  std::printf("%-52s got %+.6f bound %+.6f         %s\n", what.c_str(), got, bound, ok ? "OK" : "FAIL");
  if (!ok) failures++;
}

static void expect_true(const std::string &what, bool ok) {
  std::printf("%-52s %s\n", what.c_str(), ok ? "OK" : "FAIL");
  if (!ok) failures++;
}

// A time of the distributed clock in the year 2025, far from the epoch of that clock.
static constexpr int64_t kNow = 812345678901234567LL;

// The cycle of the core, and a camera that gives two samples in each cycle.
static constexpr double kSampleTime = 0.01;
static constexpr int64_t kFramePeriodNs = 5000000;

// A seeker, a dither and a plant, driven one cycle at a time. The plant takes the position of its input and the
// number of the cycle, and gives the measurement.
class Rig {
 public:
  ExtremumSeekerConfig cfg;
  DitherSettings dither;
  ExtremumSeekerState state;

  Rig(const ExtremumSeekerConfig &config, const DitherSettings &settings) : cfg(config), dither(settings) {}

  void start(double output) {
    seeker.start(output);
    state = ExtremumSeekerState{};
    state.running = true;
    state.output = output;
  }

  void stop() {
    seeker.stop();
    state.running = false;
  }

  // A run without samples stands for a camera that says nothing.
  template <typename Plant>
  void run(int cycles, Plant plant, bool with_samples = true) {
    for (int i = 0; i < cycles; i++) {
      seeker.begin_step(cfg, dither::amplitude(dither));
      const int64_t end = t + static_cast<int64_t>(kSampleTime * 1e9);
      for (; t_frame < end; t_frame += kFramePeriodNs) {
        const double phase = dither::phase_rad(t_frame, dither);
        const double position = state.output + dither::amplitude(dither) * std::sin(phase);
        if (with_samples) {
          seeker.add_sample(plant(position, cycle), phase);
        }
      }
      t = end;
      cycle++;
      state = seeker.step();
    }
  }

  // The largest distance from where the output is now, over a number of cycles.
  template <typename Plant>
  double excursion(int cycles, Plant plant) {
    const double from = state.output;
    double worst = 0.0;
    for (int i = 0; i < cycles; i++) {
      run(1, plant);
      worst = std::max(worst, std::abs(state.output - from));
    }
    return worst;
  }

 private:
  ExtremumSeeker seeker{kSampleTime};
  int64_t t = kNow;
  int64_t t_frame = kNow;
  int cycle = 0;
};

// Runs one seeker against a plant for a number of cycles and gives the state it ends in.
template <typename Plant>
static ExtremumSeekerState seek(const ExtremumSeekerConfig &cfg, const DitherSettings &dither, double start, int cycles,
                                Plant plant) {
  Rig rig(cfg, dither);
  rig.start(start);
  rig.run(cycles, plant);
  return rig.state;
}

// A parabola around an optimum. A curvature above zero makes a minimum, and one below zero makes a maximum.
static auto parabola(double optimum, double curvature, double value_at_optimum) {
  return [=](double position, int) {
    const double distance = position - optimum;
    return value_at_optimum + curvature * distance * distance;
  };
}

static DitherSettings sine(double frequency_hz, float amplitude) {
  DitherSettings dither;
  dither.mode = DitherSettings::kSine;
  dither.period_ns = dither::period_from_frequency(frequency_hz);
  dither.amplitude = amplitude;
  return dither;
}

static void check_dither() {
  // 7.3 Hz gives no whole quantity of nanoseconds. The period is the nearest one, and the frequency follows from it.
  const int64_t period = dither::period_from_frequency(7.3);
  expect_near("7.3 Hz gives a period of 136986301 ns", static_cast<double>(period), 136986301.0, 0.0);
  expect_near("and that period gives 7.3 Hz back", dither::frequency_from_period(period), 7.3, 1e-7);

  const DitherSettings dither = sine(7.3, 1.0f);

  // The same formula as Dither.TcPOU of the TwinCAT project, on a time far from the epoch of the clock.
  const double want =
      2.0 * std::numbers::pi * static_cast<double>(kNow % dither.period_ns) / static_cast<double>(dither.period_ns);
  expect_near("the phase is the one of the PLC", dither::phase_rad(kNow, dither), want, 0.0);

  // A whole period later the phase is the same, thus the two sides stay together for all time.
  expect_near("the phase repeats after one period", dither::phase_rad(kNow + dither.period_ns, dither),
              dither::phase_rad(kNow, dither), 1e-12);

  // A period that is one nanosecond longer walks away from it. This is why the period, and not the frequency, is the
  // setting that both sides share.
  DitherSettings drifted = dither;
  drifted.period_ns += 1;
  const double error = std::abs(dither::phase_rad(kNow, drifted) - dither::phase_rad(kNow, dither));
  expect_true("one nanosecond of difference loses the phase", error > 0.1);

  DitherSettings off = dither;
  off.mode = DitherSettings::kOff;
  expect_near("an amplitude of zero when the dither is off", dither::amplitude(off), 0.0, 0.0);
}

static void check_minimum() {
  ExtremumSeekerConfig cfg;
  cfg.direction = kMinimum;
  cfg.ki = 0.05f;
  cfg.lowpass_tau_s = 0.5f;
  cfg.limit = 5.0f;
  const DitherSettings dither = sine(5.0, 0.05f);

  const auto from_above = seek(cfg, dither, 3.5, 3000, parabola(3.0, 4.0, 0.01));
  expect_near("a minimum from above", from_above.output, 3.0, 5e-3);

  const auto from_below = seek(cfg, dither, 2.6, 3000, parabola(3.0, 4.0, 0.01));
  expect_near("a minimum from below", from_below.output, 3.0, 5e-3);

  expect_true("and it is not at its limit", !from_above.at_limit);
  expect_true("and it counted every sample", from_above.sample_count == 3000 * 2);
}

// A maximum, and without the division by the mean. The gain of the loop then holds the size of the measurement, thus
// a plant of another size takes another gain.
static void check_maximum() {
  ExtremumSeekerConfig cfg;
  cfg.direction = kMaximum;
  cfg.normalise = false;
  cfg.ki = 0.125f;  // the gain of this plant is 8 per um, thus the loop closes well below the low pass
  const DitherSettings dither = sine(5.0, 0.05f);

  const auto state = seek(cfg, dither, 3.5, 3000, parabola(3.0, -4.0, 10.0));
  expect_near("a maximum", state.output, 3.0, 5e-3);
}

static void check_no_dither() {
  ExtremumSeekerConfig cfg;
  cfg.ki = 0.05f;
  DitherSettings dither = sine(5.0, 0.05f);
  dither.mode = DitherSettings::kOff;

  const auto state = seek(cfg, dither, 3.5, 500, parabola(3.0, 4.0, 0.01));
  expect_near("no dither gives no gradient", state.gradient, 0.0, 0.0);
  expect_near("and the output stays where it started", state.output, 3.5, 0.0);
}

// The limit holds the output, and the integral does not wind up behind it: when the optimum moves to the other side,
// the output comes back at once.
static void check_limit() {
  ExtremumSeekerConfig cfg;
  cfg.ki = 0.05f;
  cfg.limit = 0.05f;
  const DitherSettings dither = sine(5.0, 0.05f);
  const int kFlip = 1500;

  const auto plant = [](double position, int cycle) {
    const double optimum = cycle < kFlip ? 3.0 : 3.6;
    const double distance = position - optimum;
    return 0.01 + 4.0 * distance * distance;
  };

  const auto at_limit = seek(cfg, dither, 3.5, kFlip, plant);
  expect_true("the limit stops the output", at_limit.at_limit);
  expect_near("and it holds it at the limit", at_limit.offset, -0.05, 1e-9);

  // The output sits at the limit for 15 s. An integral that wound up over that time would need seconds to come
  // back, and 2 s after the optimum moves the output would still be at the limit.
  const auto after = seek(cfg, dither, 3.5, kFlip + 200, plant);
  expect_true("the output leaves the limit at once", after.offset > -0.045);
}

// A run starts with the low pass of the run before. The output holds until that estimate is gone, so that the plant
// of the last run does not take the output of this one anywhere.
static void check_restart() {
  ExtremumSeekerConfig cfg;
  cfg.ki = 0.05f;
  const DitherSettings dither = sine(5.0, 0.05f);
  const auto first = parabola(3.0, 4.0, 0.01);
  const auto second = parabola(5.0, 400.0, 1.0);  // a hundred times as bright, and its optimum is somewhere else
  Rig rig(cfg, dither);

  rig.start(3.5);
  rig.run(2000, first);
  rig.stop();
  rig.run(100, first);

  rig.start(6.0);
  rig.run(100, second);
  expect_near("a new run holds its output while it settles", rig.state.output, 6.0, 0.0);
  expect_true("and it says that it settles", rig.state.settling);

  rig.run(3000, second);
  expect_near("and then it finds the minimum of the new plant", rig.state.output, 5.0, 5e-3);
  expect_true("and it counts the samples of this run only", rig.state.sample_count == 3100 * 2);
}

// A new dither amplitude changes what the demodulation gives, and the low pass carries the amplitude of before for
// its settling time. Neither may move the output.
static void check_amplitude_change() {
  ExtremumSeekerConfig cfg;
  cfg.ki = 0.05f;
  const auto plant = parabola(3.0, 4.0, 0.01);
  Rig rig(cfg, sine(5.0, 0.05f));

  rig.start(3.5);
  rig.run(3000, plant);
  expect_near("the seeker sits at the minimum", rig.state.output, 3.0, 5e-3);

  rig.dither.amplitude = 0.005f;  // a tenth of the amplitude, while the seeker runs
  expect_below("a tenth of the amplitude moves the output by", rig.excursion(500, plant), 5e-3);

  rig.dither.amplitude = 0.2f;  // and forty times as much
  expect_below("forty times the amplitude moves the output by", rig.excursion(500, plant), 5e-3);
}

// The demodulation phase and the low pass are the other two parts of the estimate. A change of either holds the
// output as well.
static void check_settings_change() {
  ExtremumSeekerConfig cfg;
  cfg.ki = 0.05f;
  const auto plant = parabola(3.0, 4.0, 0.01);
  Rig rig(cfg, sine(5.0, 0.05f));

  rig.start(3.5);
  rig.run(3000, plant);

  rig.cfg.demod_phase_rad = 1.0f;
  expect_below("a new demodulation phase moves the output by", rig.excursion(200, plant), 5e-3);

  rig.cfg.lowpass_tau_s = 2.0f;
  expect_below("a new low pass moves the output by", rig.excursion(200, plant), 5e-3);
}

// The measurement stops. The seeker holds the setpoint instead of driving it on with the newest gradient it had.
static void check_no_samples() {
  ExtremumSeekerConfig cfg;
  cfg.ki = 0.05f;
  cfg.sample_timeout_s = 0.2f;
  const auto plant = parabola(3.0, 4.0, 0.01);
  Rig rig(cfg, sine(5.0, 0.05f));

  rig.start(3.5);
  rig.run(1000, plant);  // on its way to the minimum, thus its gradient is not zero
  const double moving = rig.state.output;

  rig.run(100, plant, false);  // 1 s without a sample, thus past the timeout
  expect_true("a gap in the measurement stops the output", rig.state.stale);
  const double frozen = rig.state.output;
  expect_below("and it only carries on over the timeout", std::abs(frozen - moving), 0.03);

  rig.run(1000, plant, false);
  expect_near("and it stays there for as long as the gap lasts", rig.state.output, frozen, 0.0);

  rig.run(3000, plant);
  expect_near("and it finds the minimum when the samples come back", rig.state.output, 3.0, 5e-3);
}

// A measurement that goes through zero, as a photometry with the background taken off does at a dark fringe. The
// division by the mean must not turn the sign of the gradient over, and the rate must hold the output where that
// division makes the gradient grow without bound.
static void check_measurement_through_zero() {
  ExtremumSeekerConfig cfg;
  cfg.ki = 0.05f;
  cfg.max_rate = 1.0f;
  const auto plant = parabola(3.0, 4.0, -0.02);  // the measurement is below zero over 0.14 um around the minimum
  Rig rig(cfg, sine(5.0, 0.05f));

  rig.start(3.5);
  rig.run(3000, plant);
  expect_below("a measurement through zero keeps the output near", std::abs(rig.state.output - 3.0), 0.1);
  expect_true("and the output does not run away", !rig.state.at_limit);
}

// The rate holds the output back, whatever the gradient says.
static void check_max_rate() {
  ExtremumSeekerConfig cfg;
  cfg.ki = 0.05f;
  cfg.max_rate = 0.05f;
  cfg.normalise = false;
  Rig rig(cfg, sine(5.0, 0.05f));

  // A plant of a gradient far above what this rate allows: the output moves at the rate and no faster.
  rig.start(3.5);
  rig.run(1000, parabola(3.0, 4000.0, 1.0));
  expect_below("the output moves no faster than the rate", std::abs(rig.state.output - 3.5), 0.05 * 10.0);
  expect_true("and it says that the rate holds it back", rig.state.at_max_rate);
}

int main() {
  check_dither();
  check_minimum();
  check_maximum();
  check_no_dither();
  check_limit();
  check_restart();
  check_amplitude_change();
  check_settings_change();
  check_no_samples();
  check_measurement_through_zero();
  check_max_rate();
  std::printf("%s\n", failures == 0 ? "all checks passed" : "there are failures");
  return failures == 0 ? 0 : 1;
}
