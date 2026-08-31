#pragma once

#include <array>

#include "algorithms/IirFilter.hpp"

/*
 * @brief Implements a Proportional-Integral (PI) controller.
 *
 * The transfer function
 * C(s) = Kp + Ki/s
 * is approximated via a bilinear transform using an IIR filter.
 */
template <typename T>
class PIController {
 public:
  PIController(T Kp, T Ki, T t) : Kp(Kp), Ki(Ki), t(t) { updateCoefficients(); }

  T update(T error) {
    T output = iir_filter.filter(error);
    return output;
  }

  void setGains(T newKp, T newKi) {
    Kp = newKp;
    Ki = newKi;
    updateCoefficients();
  }

  void reset_Iir_filter() { iir_filter.reset(); }

 private:
  void updateCoefficients() {
    b[0] = Kp + Ki * t / 2;
    b[1] = -Kp + Ki * t / 2;
    a[0] = 1;
    iir_filter.set_coefficients(b, a);
  }

  T Kp;  // proportional gain
  T Ki;  // integral gain
  T t;   // sampling time

  std::array<T, 2> b;  // feedforward filter coefficients
  std::array<T, 1> a;  // feedback filter coefficients

  IirFilter<T, 2> iir_filter{b, a};
};
