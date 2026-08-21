#pragma once

#include <cstdint>
#include <numbers>

#include "data/Dither.hpp"

// The wave of a dither at any time of the clock that carries it. Dither.TcPOU of the TwinCAT project holds the same
// formula, thus the PC demodulates a measurement with the phase that the PLC had when the measurement was taken.
namespace dither {

// The phase at the time t, in radians. The time and the period are in nanoseconds, in the same clock.
inline double phase_rad(int64_t t, const DitherSettings &settings) {
  if (settings.period_ns <= 0) {
    return 0.0;
  }
  const double turn = static_cast<double>(t % settings.period_ns) / static_cast<double>(settings.period_ns);
  return 2.0 * std::numbers::pi * turn + static_cast<double>(settings.phase_rad);
}

// The amplitude that the plant sees. It is zero when the dither is off, and a seeker then finds no gradient.
inline double amplitude(const DitherSettings &settings) {
  const bool on = settings.mode == DitherSettings::kSine && settings.period_ns > 0;
  return on ? static_cast<double>(settings.amplitude) : 0.0;
}

// The period is a whole quantity of nanoseconds, thus the frequency that comes back is not always the one asked for.
inline int64_t period_from_frequency(double frequency_hz) {
  if (!(frequency_hz > 0.0)) {
    return 0;
  }
  return static_cast<int64_t>(1.0e9 / frequency_hz + 0.5);
}

inline double frequency_from_period(int64_t period_ns) {
  return period_ns > 0 ? 1.0e9 / static_cast<double>(period_ns) : 0.0;
}

}  // namespace dither
