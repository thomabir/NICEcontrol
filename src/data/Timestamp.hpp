#pragma once

#include <cstdint>

// When a measurement was taken, in the two clocks of the program.
// Both count nanoseconds from 2000-01-01 00:00.
struct Timestamp {
  int64_t t_PC = 0;
  int64_t t_DC = 0;
};

// One measurement: when it was taken, and what was measured.
template <typename T>
struct Measurement {
  Timestamp time;
  T value;
};
