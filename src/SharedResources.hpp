#pragma once

#include "MeasurementT.hpp"
#include "TSCircularBuffer.hpp"

struct MetrologyResources {
  TSCircularBuffer<MeasurementT<int, int>> adc_queues[10];
};

struct SharedResources {
  MetrologyResources metrology;
};
