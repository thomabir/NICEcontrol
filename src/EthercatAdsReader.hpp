#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <thread>

#include "EthercatAdsResources.hpp"
#include "PlcSample.hpp"
#include "SPMCRingBuffer.hpp"

static constexpr size_t PLC_BUF_SIZE = 512;  // must match PLC's STREAM_BUF_SIZE
#pragma pack(push, 1)
struct PlcBuffer {
  uint32_t buffer_no;  // most recent sample index
  PlcSample samples[PLC_BUF_SIZE];
};
#pragma pack(pop)
static_assert(sizeof(PlcBuffer) == 4 + sizeof(PlcSample) * PLC_BUF_SIZE,
              "PlcBuffer size mismatch - check packing and constants");

class EthercatAdsReader {
 public:
  EthercatAdsReader(EthercatAdsResources &resources) : res(resources) { setup(); }

  void start() {
    reader_thread = std::jthread([this](std::stop_token st) {
      while (!st.stop_requested()) {
        read_ads_package();
        std::this_thread::sleep_for(std::chrono::milliseconds(15));  // faster than screen refresh rate
      }
    });
  }

  void request_stop() {
    if (reader_thread.joinable()) {
      reader_thread.request_stop();
    }
  }

 private:
  std::jthread reader_thread;
  EthercatAdsResources &res;
  std::atomic<bool> record_data = false;  // flag to indicate if data should be recorded
  std::ofstream file;                     // file to record EtherCAT data

  uint32_t last_sample_no = 0;
  bool last_sample_initialized = false;

  int setup() { return 0; }
  void read_ads_package() {
    auto buf = res.plc().read<PlcBuffer>("MAIN.send_buffer");

    const uint32_t oldest_idx = (buf.buffer_no + 1) % PLC_BUF_SIZE;
    size_t start_j = 0;
    if (!last_sample_initialized) {
      const auto &first = buf.samples[oldest_idx];
      ring_push(first);
      last_sample_no = first.sample_no;
      last_sample_initialized = true;
      start_j = 1;
    }

    for (size_t j = start_j; j < PLC_BUF_SIZE; ++j) {
      const auto &s = buf.samples[(oldest_idx + j) % PLC_BUF_SIZE];
      if (s.sample_no <= last_sample_no) {
        continue;
      }
      if (s.sample_no != last_sample_no + 1) {
        std::cout << "Gap detected at sample " << j << ": expected sample_no=" << (last_sample_no + 1)
                  << ", got sample_no=" << s.sample_no << "\n";
      }
      ring_push(s);
      last_sample_no = s.sample_no;
    }

    // std::cout << "Read buffer_no=" << buf.buffer_no << ", latest sample_no=" << last_sample_no << "\n";
  }

  void ring_push(const PlcSample &sample) { res.data.push(sample); }
};
