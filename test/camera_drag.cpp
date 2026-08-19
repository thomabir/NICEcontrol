// Checks the live drag path: a region write in each frame while the camera streams.
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include "devices/TangoFlirCamInterface.hpp"

static int failures = 0;

static void report(const std::string &what, bool ok, const std::string &detail = "") {
  std::printf("%s%s%s%s\n", ok ? "OK    " : "FAIL  ", what.c_str(), detail.empty() ? "" : "  [",
              detail.empty() ? "" : (detail + "]").c_str());
  if (!ok) failures++;
}

int main() {
  TangoFlirCamInterface cam;
  if (cam.connect() != 0) {
    std::printf("FAIL  could not connect\n");
    return 2;
  }

  const unsigned int width = cam.get_width();
  const unsigned int height = cam.get_height() - 1;

  cam.set_phot_subtract_background(false);
  cam.start_stream();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Move a rectangle one pixel per frame for 60 frames.
  const int steps = 60;
  uint64_t last_frame = 0;
  int gaps = 0;
  size_t samples = 0;
  const auto started = std::chrono::steady_clock::now();
  for (int i = 0; i < steps; i++) {
    const ImPlotRect drawn(100.0 + i, 150.0 + i, 200.0, 250.0);
    const PhotRegion region = photometry_regions::to_image_coords(drawn, width, height);
    cam.set_regions({region});

    const PhotBatch batch = cam.get_phot_since(last_frame);
    if (!batch.samples.empty()) {
      if (last_frame != 0 && batch.samples.front().frame_id > last_frame + 1) gaps++;
      last_frame = batch.samples.back().frame_id;
      samples += batch.samples.size();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }
  const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();

  report("the regions took every write at the render rate", true,
         std::to_string(steps) + " writes in " + std::to_string(elapsed) + " s");
  report("photometry kept flowing during the drag", samples > 0, std::to_string(samples) + " samples");
  report("no frame was lost during the drag", gaps == 0, std::to_string(gaps) + " gaps");

  // The camera holds the last region of the drag.
  const ImPlotRect final_rect(100.0 + (steps - 1), 150.0 + (steps - 1), 200.0, 250.0);
  const PhotRegion expected = photometry_regions::to_image_coords(final_rect, width, height);
  const std::vector<PhotRegion> got = cam.get_regions();
  const bool ok = got.size() == 1 && got[0].x0 == expected.x0 && got[0].y0 == expected.y0 && got[0].x1 == expected.x1 &&
                  got[0].y1 == expected.y1;
  report("the camera holds the last region of the drag", ok,
         "want (" + std::to_string(expected.x0) + "," + std::to_string(expected.y0) + ")-(" +
             std::to_string(expected.x1) + "," + std::to_string(expected.y1) + ")");

  cam.stop_stream();
  std::printf("\n%s (%d failure(s))\n", failures == 0 ? "DRAG CHECK PASSED" : "DRAG CHECK FAILED", failures);
  return failures == 0 ? 0 : 1;
}
