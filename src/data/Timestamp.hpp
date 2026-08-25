#pragma once

#include <cstdint>

/**
 * @brief When a measurement was taken, in the two clocks of the program.
 *
 * Both clocks count nanoseconds from 2000-01-01 00:00.
 */
struct Timestamp {
  int64_t t_PC = 0;     /**< the monotonic clock of the PC */
  int64_t t_DC = 0;     /**< the distributed clock of the EtherCAT bus, and a copy of t_PC while dc_good is false */
  bool dc_good = false; /**< t_DC is a time of the bus: an offset of the two clocks arrived and it is young enough */
};

/** @brief One measurement: when it was taken, and what was measured. */
template <typename T>
struct Measurement {
  Timestamp time;
  T value;
};
