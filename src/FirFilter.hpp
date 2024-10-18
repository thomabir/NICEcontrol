#pragma once

#include <array>

/**
 * @brief Implements a Finite Impulse Response (FIR) filter.
 *
 * @tparam T The type of the filter's coefficients and input/output data.
 * @tparam N The number of filter coefficients.
 */
template <typename T, int N>
class FirFilter {
 public:
  // Initialize filter with coefficients
  FirFilter(const std::array<T, N>& coefficients) : coefficients(coefficients) { buffer.fill(0); }

  // default constructor: all zeros
  FirFilter() : coefficients({0}), buffer({0}) {}

  /**
   * @brief Filters the input signal using the FIR filter.
   *
   * @param input The new input sample.
   * @returns The filtered output.
   */
  T filter(T input) {
    // shift buffer to make room for new input
    for (int i = N - 1; i > 0; --i) {
      buffer[i] = buffer[i - 1];
    }

    // add new input to buffer
    buffer[0] = input;

    // calculate output using FIR filter equation
    T output = 0;
    for (int i = 0; i < N; ++i) {
      output += coefficients[i] * buffer[i];
    }

    return output;
  }

  void reset() { buffer.fill(0); }

  void load_coefficients(const std::array<T, N>& coefficients) { this->coefficients = coefficients; }

 private:
  std::array<T, N> coefficients;  // filter coefficients
  std::array<T, N> buffer;        // buffer for input samples (0 is the most recent, N-1 is the oldest)
};