// Checks that a change of Width or Height keeps the photometry regions inside the new frame.
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

  const unsigned int start_width = cam.get_width();
  const unsigned int start_height = cam.get_height();
  std::printf("INFO  frame at start: %u x %u (Height attribute %u)\n", start_width, start_height - 1, start_height);

  // Start from a large frame.
  cam.write_width(640);
  cam.write_height(513);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  const unsigned int big_w = cam.get_width();
  const unsigned int big_h = cam.get_height() - 1;
  report("the frame grew to 640 x 512", big_w == 640 && big_h == 512,
         std::to_string(big_w) + " x " + std::to_string(big_h));

  // One region fits in the small frame, one does not.
  const std::vector<PhotRegion> regions{{10, 10, 20, 20}, {300, 300, 400, 400}};
  cam.set_regions(regions);
  report("two regions were taken at the large size", cam.get_regions().size() == 2);

  // Shrink the frame.
  cam.write_width(64);
  cam.write_height(65);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  const unsigned int small_w = cam.get_width();
  const unsigned int small_h = cam.get_height() - 1;
  report("the frame shrank to 64 x 64", small_w == 64 && small_h == 64,
         std::to_string(small_w) + " x " + std::to_string(small_h));

  const std::vector<PhotRegion> after = cam.get_regions();
  report("the region that no longer fits was dropped", after.size() == 1,
         std::to_string(after.size()) + " region(s) left");
  bool inside = true;
  for (const auto &r : after) inside = inside && r.x1 < small_w && r.y1 < small_h;
  report("every region left is inside the new frame", inside);

  // The photometry runs for the region that is left.
  cam.start_stream();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  const PhotBatch batch = cam.get_phot_since(0);
  cam.stop_stream();
  report("photometry still runs after the size change", !batch.samples.empty() && batch.n_regions == after.size(),
         std::to_string(batch.samples.size()) + " samples, " + std::to_string(batch.n_regions) + " regions");

  // Put the frame back.
  cam.write_width(start_width);
  cam.write_height(start_height);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  report("the frame was put back", cam.get_width() == start_width && cam.get_height() == start_height,
         std::to_string(cam.get_width()) + " x " + std::to_string(cam.get_height() - 1));

  std::printf("\n%s (%d failure(s))\n", failures == 0 ? "RESIZE CHECK PASSED" : "RESIZE CHECK FAILED", failures);
  return failures == 0 ? 0 : 1;
}
