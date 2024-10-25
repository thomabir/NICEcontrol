#pragma once

#include "FirFilter.hpp"
#include "Iir.h"

class NullLockin {
 public:
  NullLockin() {
    // input filters
    double sampling_rate = 128000;  // Hz
    double center = 20000;          // Hz
    double width = 1000;            // Hz
    bp1.setup(sampling_rate, center, width);
    bp2.setup(sampling_rate, center, width);

    // lockin lowpass
    double cutoff = 1000;  // Hz
    lp1.setup(sampling_rate, cutoff);
    lp2.setup(sampling_rate, cutoff);
  }

  void update_cutoff(double fcutoff) {
    lp1.setup(128000, fcutoff);
    lp2.setup(128000, fcutoff);
  }

  double process(int adc_sci_null, int adc_sci_mod) {
    double sci_null = adc_sci_null;
    double sci_mod = adc_sci_mod;

    // input filters
    sci_null = bp1.filter(sci_null);
    sci_mod = bp2.filter(sci_mod);

    // delay and hilbert
    double sci_null_delay = delay_1.filter(sci_null);
    double sci_mod_delay = delay_2.filter(sci_mod);
    double sci_mod_90 = hilbert_1.filter(sci_mod);

    // multipliers
    double x = sci_null_delay * sci_mod_delay;
    double y = sci_null_delay * sci_mod_90;

    // lockin lowpass
    x = lp1.filter(x);
    y = lp2.filter(y);

    // magnitude
    double result = std::sqrt(x * x + y * y);
    // double result = sci_null;

    return result / 1.8e6;
  }

 private:
  // input filters
  Iir::Butterworth::BandPass<4> bp1, bp2;

  // hilbert and delay
  static constexpr int num_taps = 15;
  static constexpr std::array<double, num_taps> delay_coefficients = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                                                                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  static constexpr std::array<double, num_taps> hilbert_coefficients = {
      1.2621248735966336e-18, -0.0, -0.011516887684438652,  -0.0, -0.09744164996932805, -0.0,
      -0.5859216770501672,    0.0,  0.5859216770501672,     0.0,  0.09744164996932805,  0.0,
      0.011516887684438652,   0.0,  -1.2621248735966336e-18};

  FirFilter<double, num_taps> delay_1 = FirFilter<double, num_taps>(delay_coefficients);
  FirFilter<double, num_taps> delay_2 = FirFilter<double, num_taps>(delay_coefficients);
  FirFilter<double, num_taps> hilbert_1 = FirFilter<double, num_taps>(hilbert_coefficients);

  // lockin lowpass
  Iir::Butterworth::LowPass<8> lp1, lp2;
};