#pragma once

#include <cstdint>

// The settings and the report of an extremum seeking controller. The algorithm is in algorithms/ExtremumSeeker.hpp.

// Which end of the measurement the seeker looks for. It is the sign of the step against the gradient.
enum Extremum { kMinimum = -1, kMaximum = 1 };

struct ExtremumSeekerConfig {
  int direction = kMinimum;
  float kp = 0.0f;
  float ki = 0.02f;
  float demod_phase_rad = 0.0f;  // takes out the delay from the plant to the measurement
  float lowpass_tau_s = 0.5f;    // of the low pass after the demodulation
  bool normalise = true;         // divide the gradient by the mean measurement, so that the gains stay valid
  float limit = 5.0f;            // the largest distance from the output where the seeker started
};

struct ExtremumSeekerState {
  bool running = false;
  double mean = 0.0;      // the measurement, after the low pass
  double gradient = 0.0;  // d(measurement) / d(output), from the first harmonic of the dither
  double output = 0.0;    // what the seeker asks the plant for
  double offset = 0.0;    // how far the output moved from where the seeker started
  bool at_limit = false;
  uint64_t sample_count = 0;
};
