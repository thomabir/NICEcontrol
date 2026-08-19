#pragma once

#include <cstdint>

// When a measurement was taken, in the two clocks of the program.
// t_PC is the monotonic PC clock, t_DC the EtherCAT distributed clock.
// The source of the measurement gives one of the two, and the other one follows from the offset of the clocks.
// dc_direct is true when the source gave t_DC, and false when the source gave t_PC.
struct Timestamp {
  int64_t t_PC = 0;
  int64_t t_DC = 0;
  bool dc_direct = false;
};

// One measurement: when it was taken, and what was measured.
template <typename T>
struct Measurement {
  Timestamp time;
  T value;
};
