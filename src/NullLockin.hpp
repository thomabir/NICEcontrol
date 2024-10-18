#pragma once

class NullLockin {
 public:
  NullLockin() {}

  double process(int adc_sci_null, int adc_sci_mod) { return adc_sci_null; }
};