#pragma once

#include <array>
#include <atomic>

#include "EthercatResources.hpp"
#include "MeasurementT.hpp"
#include "PiezoActuators.hpp"
#include "SPMCRingBuffer.hpp"
#include "TSCircularBuffer.hpp"

struct MetrologyResources {
  TSCircularBuffer<MeasurementT<int, int>> adc_queues[16];
};

struct SharedResources {
  MetrologyResources metrology;
  EthercatResources ethercat;
  PiezoActuators piezos;
};
