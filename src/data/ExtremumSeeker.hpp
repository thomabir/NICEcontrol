#pragma once

#include <cstdint>

// The settings and the report of an extremum seeking controller. The algorithm is in algorithms/ExtremumSeeker.hpp.

// Which end of the measurement the seeker looks for. It is the sign of the step against the gradient.
enum Extremum { kMinimum = -1, kMaximum = 1 };

struct ExtremumSeekerConfig {
  int direction = kMinimum;
  float kp = 0.0f;
  float ki = 0.02f;
  float demod_phase_rad = 0.0f;   // takes out the delay from the plant to the measurement
  float lowpass_tau_s = 0.5f;     // of the low pass after the demodulation
  bool normalise = true;          // divide the gradient by the mean measurement, so that the gains stay valid
  float limit = 5.0f;             // the largest distance from the output where the seeker started
  float max_rate = 1.0f;          // the fastest the output moves, in the unit of the output per second
  float sample_timeout_s = 1.0f;  // a gap in the measurement longer than this holds the output
};

struct ExtremumSeekerState {
  bool running = false;
  double mean = 0.0;      // the measurement, after the low pass
  double gradient = 0.0;  // d(measurement) / d(output), from the first harmonic of the dither
  double output = 0.0;    // what the seeker asks the plant for
  double offset = 0.0;    // how far the output moved from where the seeker started
  bool at_limit = false;
  bool at_max_rate = false;   // the gradient asks for more than the output is allowed to move
  bool settling = false;      // the estimate of the gradient is not ready yet, thus the output holds
  bool stale = false;         // no measurement arrived within the timeout, thus the output holds
  uint64_t sample_count = 0;  // of the run that is going on
};
