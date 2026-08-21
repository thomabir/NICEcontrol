// Checks the dither phase in src/algorithms/Dither.hpp and the seeker in src/algorithms/ExtremumSeeker.hpp.
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

static void expect_true(const std::string &what, bool ok) {
  std::printf("%-52s %s\n", what.c_str(), ok ? "OK" : "FAIL");
  if (!ok) failures++;
}

// A time of the distributed clock in the year 2025, far from the epoch of that clock.
static constexpr int64_t kNow = 812345678901234567LL;

// The cycle of the core, and a camera that gives two samples in each cycle.
static constexpr double kSampleTime = 0.01;
static constexpr int64_t kFramePeriodNs = 5000000;

// Runs the seeker against a plant for a number of cycles and gives the state it ends in.
// The plant takes the position of its input and the number of the cycle, and gives the measurement.
template <typename Plant>
static ExtremumSeekerState seek(const ExtremumSeekerConfig &cfg, const DitherSettings &dither, double start, int cycles,
                                Plant plant) {
  ExtremumSeeker seeker(kSampleTime);
  seeker.start(start);
  ExtremumSeekerState state;
  state.output = start;
  int64_t t = kNow;
  int64_t t_frame = kNow;

  for (int cycle = 0; cycle < cycles; cycle++) {
    seeker.begin_step(cfg);
    const int64_t end = t + static_cast<int64_t>(kSampleTime * 1e9);
    for (; t_frame < end; t_frame += kFramePeriodNs) {
      const double phase = dither::phase_rad(t_frame, dither);
      const double position = state.output + dither::amplitude(dither) * std::sin(phase);
      seeker.add_sample(plant(position, cycle), phase);
    }
    t = end;
    state = seeker.step(dither::amplitude(dither));
  }
  return state;
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

int main() {
  check_dither();
  check_minimum();
  check_maximum();
  check_no_dither();
  check_limit();
  std::printf("%s\n", failures == 0 ? "all checks passed" : "there are failures");
  return failures == 0 ? 0 : 1;
}
