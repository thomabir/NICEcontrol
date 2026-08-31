#pragma once

#include <array>

/**
 * @brief Implements an Infinite Impulse Response (IIR) filter.
 *
 * Transfer function:
 * H(z) = Y(z)/X(z) =  b[0] + b[1]*z^-1 + b[2]*z^-2 + ... + b[N]*z^-N
 *                    -------------------------------------------------
 *                    1 - a[0]*z^-1 - a[1]*z^-2 - ... - a[N-1]*z^-(N-1)
 *
 * Equation:
 * y[-1] = b[0]*x[0] + b[1]*x[1] + b[2]*x[2] + ... + b[N]*x[N]
 *       + a[0]*y[0] + a[1]*y[1] + ... + a[N-1]*y[N-1]
 *
 * Where:
 * - y is the output (y[-1] is the output we are currently calculating, y[0] is
 *   the previous output, y[1] is the one before that, etc.)
 * - x is the input (x[0] is current input, x[1] is previous input, etc.)
 * - b is the array of filter coefficients for the numerator (feedforward, FIR)
 * - a is the array of filter coefficients for the denominator (feedback)
 *
 * Note that in the code, a starts from zero instead of one, same for y.
 *
 * @tparam T The type of the filter's coefficients and input/output data.
 * @tparam N The number of feedforward filter coefficients.
 */
template <typename T, int N>
class IirFilter {
 public:
  // Initialize filter with coefficients
  IirFilter(const std::array<T, N>& b, const std::array<T, N - 1>& a) : a(a), b(b) {
    x.fill(0);
    y.fill(0);
  }

  /**
   * @brief Filters the input signal using the IIR filter.
   *
   * @param input The new input sample.
   * @returns The filtered output.
   */
  T filter(T input) {
    // Shift and update input buffer x
    for (int i = N - 1; i > 0; --i) {
      x[i] = x[i - 1];
    }
    x[0] = input;

    // calculate output using IIR filter equation
    T output = 0;
    for (int i = 0; i < N; ++i) {
      output += b[i] * x[i];
    }
    for (int i = 0; i < N - 1; ++i) {
      output += a[i] * y[i];
    }

    // Shift and update output buffer y
    for (int i = N - 2; i > 0; --i) {
      y[i] = y[i - 1];
    }
    y[0] = output;

    return output;
  }

  void reset() {
    x.fill(0);
    y.fill(0);
  }

  /**
   * @brief Puts the filter in the steady state that a constant input leads to.
   *
   * A filter that starts from zero needs its settling time before its output means anything. This gives it the
   * output that the input has already led to, so the first sample is as good as the ones after it.
   *
   * @param input The constant input the filter is to start from.
   */
  void preset(T input) {
    T sum_b = 0;
    T sum_a = 0;
    for (int i = 0; i < N; ++i) {
      sum_b += b[i];
    }
    for (int i = 0; i < N - 1; ++i) {
      sum_a += a[i];
    }
    x.fill(input);
    // The gain at zero frequency. A filter that integrates has none, and it stays at zero.
    y.fill(sum_a != 1 ? input * sum_b / (1 - sum_a) : 0);
  }

  void set_coefficients(const std::array<T, N>& b, const std::array<T, N - 1>& a) {
    this->b = b;
    this->a = a;
  }

 private:
  std::array<T, N - 1> a;  // filter coefficients for denominator (feedback)
  std::array<T, N> b;      // filter coefficients for numerator (feedforward, FIR)
  std::array<T, N> x;      // buffer for input samples (0 is the most recent, N-1 is the oldest)
  std::array<T, N - 1> y;  // buffer for output samples (0 is the most recent, N-1 is the oldest)
};
