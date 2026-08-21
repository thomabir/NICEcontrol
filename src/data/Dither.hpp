#pragma once

#include <cstdint>

// One dither: a wave that a controller adds to the input of a plant, so that the reaction of a measurement to that
// wave tells the controller which way to move. The PLC makes the wave for the plants that it drives.
//
// The period is a whole quantity of nanoseconds, and the phase counts from the epoch of the clock. Whoever makes the
// wave and whoever demodulates a measurement then compute the same phase from the same timestamp, at any frequency
// and for all time. A frequency in a float would not do that: the two sides round the division to a period
// differently, and one nanosecond of difference grows into many turns of phase over the size of the timestamp.
struct DitherSettings {
  enum Mode { kOff = 0, kSine = 1 };

  int mode = kOff;
  int64_t period_ns = 0;
  float phase_rad = 0.0f;  // at the epoch of the clock
  float amplitude = 0.0f;  // in the unit of the plant input
};
