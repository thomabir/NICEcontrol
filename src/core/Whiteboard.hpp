#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "core/DcClockEstimate.hpp"
#include "data/PhotometryRegions.hpp"
#include "data/PlcSample.hpp"
#include "data/SPMCRingBuffer.hpp"
#include "devices/TangoFlirCamInterface.hpp"

// The whiteboard is the public data of the core. Every application writes its own part and reads any other part.
// It has three kinds of data:
//   Streams  the sample history. Any number of readers subscribe once and drain at their own pace.
//   State    the latest values. The core publishes a snapshot of the state after each cycle.
//   Clock    the DC time of any PC time. Any thread asks at any time, not once per cycle.

// One timepoint of the 16 metrology ADC channels.
struct AdcSample {
  int counter = 0;
  std::array<int, 16> value{};
};

struct CoreState {
  uint64_t cycle = 0;
  double time_s = 0.0;
  double cycle_ms = 0.0;
  double clock_ms = 0.0;
  double metrology_ms = 0.0;
  double plc_ms = 0.0;
  double tiptilt_ms = 0.0;
  double camera_ms = 0.0;
  double devices_ms = 0.0;
  uint64_t overruns = 0;
};

// The EtherCAT distributed clock, and the filter that gives the DC time of any PC time.
struct ClockState {
  bool card_open = false;      // the card is open and the esd stack runs
  bool clock_present = false;  // the maindevice distributes the time, and the newest sample is fresh
  bool locked = false;         // the filter has an estimate
  int al_state = 0;            // 1 INIT, 2 PREOP, 4 SAFEOP, 8 OP
  uint64_t dc_ns = 0;          // the distributed clock of the newest pair, from 2000-01-01 00:00
  double read_span_us = 0.0;   // the length of the read that latched the value, the uncertainty of the pair
  double age_ms = 0.0;         // the delay from the pair to the cycle that read it
  double rate_ppm = 0.0;       // how much faster the DC clock runs than the PC clock
  double offset_sd_ns = 0.0;   // the uncertainty of the estimate
  double rate_sd_ppb = 0.0;    // the uncertainty of the rate
  double error_ns = 0.0;       // the estimate of the newest pair minus the pair itself
  uint64_t sample_count = 0;
  uint64_t rejected_count = 0;  // the pairs that the gate of the filter refused
};

struct MetrologyState {
  bool socket_open = false;
  int counter = 0;
  uint64_t sample_count = 0;
  uint64_t dropped_packets = 0;
};

// The optical path difference, measured and controlled by the PLC.
struct OpdState {
  bool connected = false;
  uint32_t sample_no = 0;
  double timestamp_s = 0.0;
  float opd_um = 0.0f;
  float dl_pos_um = 0.0f;
  float dl_cmd_um = 0.0f;
  uint64_t sample_count = 0;
  uint64_t gaps = 0;
};

// The two lateral beam positions, set by the PI tip/tilt stages.
struct TipTiltState {
  bool connected = false;
  float x1 = 0.0f;
  float y1 = 0.0f;
  float x2 = 0.0f;
  float y2 = 0.0f;
};

struct CameraState {
  bool connected = false;
  double framerate = 0.0;
  double integration_time_ms = 0.0;
  unsigned int width = 0;
  unsigned int height = 0;
  std::string filename;
  bool subtract_background = false;
  std::vector<PhotRegion> regions;
  std::vector<std::string> device_commands;
  size_t n_regions = 0;
  std::array<double, kMaxPhotRegions> values{};
  uint64_t frame_count = 0;
};

struct TangoDeviceState {
  bool connected = false;
  std::vector<std::string> device_commands;
};

struct Snapshot {
  CoreState core;
  ClockState clock;
  MetrologyState metrology;
  OpdState opd;
  TipTiltState tiptilt;
  CameraState camera;
  TangoDeviceState shutter;
  TangoDeviceState ndfilter;
};

class Whiteboard {
 public:
  SPMCRingBuffer<AdcSample, 20000> adc;
  SPMCRingBuffer<PlcSample, 20000> plc;
  SPMCRingBuffer<PhotSample, 20000> phot;

  // ClockApp writes this, and any thread reads it at any time.
  DcClockEstimate dc_clock;

  // The applications write here during the cycle.
  Snapshot state;

  void publish() {
    std::lock_guard<std::mutex> lock(mutex);
    published = state;
  }

  Snapshot snapshot() {
    std::lock_guard<std::mutex> lock(mutex);
    return published;
  }

 private:
  Snapshot published;
  std::mutex mutex;
};
