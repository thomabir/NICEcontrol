#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <vector>

#include "data/PhotometryRegions.hpp"

// The photometry of one frame, as GetPhotSince gives it.
struct PhotSample {
  uint64_t frame_id = 0;
  std::chrono::steady_clock::time_point t_mono;  // when the server got the frame, in the monotonic clock of the PC
  double t_cam_ns = 0.0;                         // the clock of the camera, as the frame carries it
  std::array<double, kMaxPhotRegions> values{};
};

// The reply of one GetPhotSince call.
struct PhotBatch {
  size_t n_regions = 0;
  std::vector<PhotSample> samples;
};
