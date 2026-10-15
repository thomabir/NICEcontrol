// Exercises the frame sync calls of the GUI client code (src/devices/TangoFlirCamInterface.hpp) against the live
// camera. Each write stops the stream, writes, and starts the stream again, as the FLIR panel does.
// At the end it puts back the sync settings that the camera had at the start.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>

#include "devices/TangoFlirCamInterface.hpp"

static int failures = 0;

// A delay counts as equal when it is this close to the written value, because the camera stores it as a 32 bit float.
static constexpr double kDelayTolerance_us = 1e-3;

// The wait for new frames after a write.
static constexpr int kFrameWait_ms = 3000;

static void report(const std::string &what, bool ok, const std::string &detail = "") {
  std::printf("%s%s%s%s\n", ok ? "OK    " : "FAIL  ", what.c_str(), detail.empty() ? "" : "  [",
              detail.empty() ? "" : (detail + "]").c_str());
  if (!ok) failures++;
}

// The newest frame number that the camera has measured, or last_frame when no new frame came.
static uint64_t newest_frame(TangoFlirCamInterface &cam, uint64_t last_frame) {
  const PhotBatch batch = cam.get_phot_since(last_frame);
  return batch.samples.empty() ? last_frame : batch.samples.back().frame_id;
}

// Expect the stream to run again after a write.
static void check_stream(TangoFlirCamInterface &cam, const std::string &label) {
  const uint64_t start_frame = newest_frame(cam, 0);
  uint64_t frame = start_frame;
  const auto start = std::chrono::steady_clock::now();
  while (frame == start_frame && std::chrono::steady_clock::now() - start < std::chrono::milliseconds(kFrameWait_ms)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    frame = newest_frame(cam, start_frame);
  }
  report("frames arrive after " + label, frame != start_frame,
         "frame " + std::to_string(start_frame) + " -> " + std::to_string(frame));
}

int main() {
  TangoFlirCamInterface cam;
  if (cam.connect() != 0) {
    std::printf("FAIL  could not connect to the camera\n");
    return 2;
  }
  report("connected", cam.is_connected());

  // One region, so that GetPhotSince gives a sample for each frame.
  const unsigned int width = cam.get_width();
  const unsigned int height = cam.get_height() - 1;  // the published frame drops the metadata row
  cam.set_regions({{0, 0, width / 2, height / 2}});

  const std::string source_start = cam.get_sync_source();
  const std::string mode_start = cam.get_sync_mode();
  const std::string polarity_start = cam.get_sync_polarity();
  const double delay_start_us = cam.get_sync_delay_us();
  std::printf("INFO  sync at start: source %s, mode %s, polarity %s, delay %g us\n", source_start.c_str(),
              mode_start.c_str(), polarity_start.c_str(), delay_start_us);

  cam.write_sync_source("Internal");
  report("sync source reads back as Internal", cam.get_sync_source() == "Internal", cam.get_sync_source());

  for (const std::string mode : {"Readout", "Integration"}) {
    cam.write_sync_mode(mode);
    report("sync mode reads back as " + mode, cam.get_sync_mode() == mode, cam.get_sync_mode());
    check_stream(cam, "sync mode " + mode);
  }

  for (const std::string polarity : {"ActiveLow", "ActiveHigh"}) {
    cam.write_sync_polarity(polarity);
    report("sync polarity reads back as " + polarity, cam.get_sync_polarity() == polarity, cam.get_sync_polarity());
    check_stream(cam, "sync polarity " + polarity);
  }

  for (const double delay_us : {100.0, 0.0}) {
    cam.write_sync_delay_us(delay_us);
    const double back_us = cam.get_sync_delay_us();
    report("sync delay reads back as " + std::to_string(delay_us) + " us",
           std::fabs(back_us - delay_us) < kDelayTolerance_us, std::to_string(back_us) + " us");
    check_stream(cam, "sync delay " + std::to_string(delay_us) + " us");
  }

  cam.write_sync_source(source_start);
  cam.write_sync_mode(mode_start);
  cam.write_sync_polarity(polarity_start);
  cam.write_sync_delay_us(delay_start_us);
  const bool restored = cam.get_sync_source() == source_start && cam.get_sync_mode() == mode_start &&
                        cam.get_sync_polarity() == polarity_start &&
                        std::fabs(cam.get_sync_delay_us() - delay_start_us) < kDelayTolerance_us;
  report("sync settings restored", restored);

  cam.stop_stream();
  std::printf("\n%s (%d failure(s))\n", failures == 0 ? "GUI SYNC CHECK PASSED" : "GUI SYNC CHECK FAILED", failures);
  return failures == 0 ? 0 : 1;
}
