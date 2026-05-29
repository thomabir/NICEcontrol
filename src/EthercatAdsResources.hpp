#pragma once

#include <memory>
#include <unordered_map>

#include "PlcConnection.hpp"
#include "PlcSample.hpp"
#include "SPMCRingBuffer.hpp"

struct EthercatAdsResources {
  SPMCRingBuffer<PlcSample, 10000> data;
  PlcConnection::Config plc_config{"192.168.88.21", {5, 168, 39, 125, 1, 1}, {1, 2, 3, 4, 5, 6}};

  PlcConnection &plc() {
    thread_local std::unordered_map<const EthercatAdsResources *, std::unique_ptr<PlcConnection>> connections;
    auto it = connections.find(this);
    if (it == connections.end()) {
      it = connections.emplace(this, std::make_unique<PlcConnection>(plc_config)).first;
    }
    return *it->second;
  }
};
