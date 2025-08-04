#pragma once

#include <array>
#include <atomic>
#include <string>

#include "EthercatResources.hpp"
#include "MeasurementT.hpp"
#include "NullLockin.hpp"
#include "PiezoActuators.hpp"
#include "SPMCRingBuffer.hpp"
#include "SensorData.hpp"
#include "TSCircularBuffer.hpp"

struct MetrologyResources {
  // control loop
  // std::atomic<bool> reset_phase_unwrap = false;

  // data queues
  TSCircularBuffer<MeasurementT<int, int>> adc_queues[15];
  SPMCRingBuffer<SensorData, 100000> sensorDataQueue;

  // science beam
  NullLockin null_lockin;
};

struct SharedResources {
  MetrologyResources metrology;
  EthercatResources ethercat;
  PiezoActuators piezos;
};
