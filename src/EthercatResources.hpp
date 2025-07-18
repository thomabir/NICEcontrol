#pragma once

#include "EthercatData.hpp"
#include "SPMCRingBuffer.hpp"

struct EthercatResources {
  SPMCRingBuffer<EthercatData, 10000> data;
};
