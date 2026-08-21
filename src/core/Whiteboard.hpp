#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "core/Clocks.hpp"
#include "data/ExtremumSeeker.hpp"
#include "data/PhotometryRegions.hpp"
#include "data/PlcSample.hpp"
#include "data/SPMCRingBuffer.hpp"
#include "data/Timestamp.hpp"
#include "devices/TangoFlirCamInterface.hpp"

// The whiteboard is the public data of the core. Every application writes its own part and reads any other part.
// It has three kinds of data:
//   Streams  the sample history. Any number of readers subscribe once and drain at their own pace.
//   State    the latest values. The core publishes a snapshot of the state after each cycle.
//   Clocks   the two clocks. Any thread asks for a time at any instant, not once per cycle.

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
  double opd_seeker_ms = 0.0;
  uint64_t overruns = 0;
};

// The EtherCAT distributed clock, and the filter that gives the DC time of any PC time.
struct ClockState {
  bool card_open = false;      // the card is open and the esd stack runs
  bool clock_present = false;  // the maindevice distributes the time, and the newest sample is fresh
  bool locked = false;         // the filter has an estimate
  int al_state = 0;            // 1 INIT, 2 PREOP, 4 SAFEOP, 8 OP
  int64_t dc_ns = 0;           // the distributed clock of the newest pair, from 2000-01-01 00:00
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
  Timestamp time;  // of the newest sample of the PLC
  float opd_um = 0.0f;
  float dl_pos_um = 0.0f;
  float dl_cmd_um = 0.0f;
  float setpoint_um = 0.0f;  // the setpoint that the core last sent, from the user interface or from the optimizer
  uint64_t sample_count = 0;
  uint64_t gaps = 0;
};

// The extremum seeker on the OPD setpoint. Its output is the setpoint in um and its measurement is the intensity of
// one photometry region.
struct OpdSeekerState {
  int region = 0;
  ExtremumSeekerState seeker;
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
  Timestamp time;  // when the core took this snapshot
  CoreState core;
  ClockState clock;
  MetrologyState metrology;
  OpdState opd;
  OpdSeekerState opd_seeker;
  TipTiltState tiptilt;
  CameraState camera;
  TangoDeviceState shutter;
  TangoDeviceState ndfilter;
};

class Whiteboard {
 public:
  SPMCRingBuffer<Measurement<AdcSample>, 20000> adc;
  SPMCRingBuffer<Measurement<PlcSample>, 20000> plc;
  SPMCRingBuffer<Measurement<PhotSample>, 20000> phot;

  // ClockApp writes the offset, and any thread asks for a time.
  Clocks clocks;

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
