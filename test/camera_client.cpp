// Exercises the GUI client code (src/TangoFlirCamInterface.hpp) against the live camera.
// It is the same code that the FLIR panel calls.
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include "../src/TangoFlirCamInterface.hpp"

static int failures = 0;

static void report(const std::string &what, bool ok, const std::string &detail = "") {
  std::printf("%s%s%s%s\n", ok ? "OK    " : "FAIL  ", what.c_str(), detail.empty() ? "" : "  [",
              detail.empty() ? "" : (detail + "]").c_str());
  if (!ok) failures++;
}

int main() {
  TangoFlirCamInterface cam;
  if (cam.connect() != 0) {
    std::printf("FAIL  could not connect to the camera\n");
    return 2;
  }
  report("connected", cam.is_connected());
  cam.stop_stream();
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  const unsigned int width = cam.get_width();
  const unsigned int height = cam.get_height() - 1;  // the published frame drops the metadata row
  std::printf("INFO  frame %u x %u\n", width, height);

  // The region round trip through the GUI helpers. The regions come from the frame size.
  const std::vector<PhotRegion> sent{{1, 1, width / 8, height / 8},
                                     {width / 4, height / 4, width / 2, height / 2},
                                     {width - 5, height - 5, width - 1, height - 1}};
  cam.set_regions(sent);
  const std::vector<PhotRegion> got = cam.get_regions();
  bool same = got.size() == sent.size();
  for (size_t i = 0; i < sent.size() && same; i++)
    same = got[i].x0 == sent[i].x0 && got[i].y0 == sent[i].y0 && got[i].x1 == sent[i].x1 && got[i].y1 == sent[i].y1;
  report("set_regions then get_regions gives the same regions", same,
         "sent " + std::to_string(sent.size()) + ", got " + std::to_string(got.size()));

  // The plot conversion over the trip to the camera and back.
  const ImPlotRect drawn(width * 0.25, width * 0.5, height * 0.25, height * 0.5);
  const PhotRegion converted = photometry_regions::to_image_coords(drawn, width, height);
  cam.set_regions({converted});
  const std::vector<PhotRegion> after = cam.get_regions();
  const bool convert_ok = after.size() == 1 && after[0].x0 == converted.x0 && after[0].y0 == converted.y0 &&
                          after[0].x1 == converted.x1 && after[0].y1 == converted.y1;
  report("a drawn rectangle survives the trip to the camera", convert_ok,
         "(" + std::to_string(converted.x0) + "," + std::to_string(converted.y0) + ")-(" +
             std::to_string(converted.x1) + "," + std::to_string(converted.y1) + ")");

  // Back to three regions.
  cam.set_regions(sent);
  cam.set_phot_subtract_background(false);
  report("PhotSubtractBackground reads back as set", !cam.get_phot_subtract_background());

  cam.start_stream();
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  // Poll the way the render loop does.
  uint64_t last_frame = 0;
  size_t total = 0;
  int gaps = 0;
  int polls = 0;
  for (int i = 0; i < 20; i++) {
    const PhotBatch batch = cam.get_phot_since(last_frame);
    polls++;
    if (!batch.samples.empty()) {
      if (last_frame != 0 && batch.samples.front().frame_id > last_frame + 1) gaps++;
      for (size_t s = 1; s < batch.samples.size(); s++)
        if (batch.samples[s].frame_id != batch.samples[s - 1].frame_id + 1) gaps++;
      last_frame = batch.samples.back().frame_id;
      total += batch.samples.size();
      if (batch.n_regions != sent.size()) {
        report("each batch reports three regions", false, std::to_string(batch.n_regions));
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }
  report("polling gave samples", total > 0,
         std::to_string(total) + " samples over " + std::to_string(polls) + " polls");
  report("no frame was lost between polls", gaps == 0, std::to_string(gaps) + " gaps");

  // Both image products at the published size.
  const Image<int> raw = cam.get_image();
  const Image<int> subtracted = cam.get_image_bg_sub();
  report("get_image gives the published size", raw.width == width && raw.height == height,
         std::to_string(raw.width) + " x " + std::to_string(raw.height));
  report("get_image_bg_sub gives the published size", subtracted.width == width && subtracted.height == height,
         std::to_string(subtracted.width) + " x " + std::to_string(subtracted.height));

  long negatives = 0;
  for (int v : subtracted.data)
    if (v < 0) negatives++;
  report("the subtracted image keeps its negative values", negatives > 0,
         std::to_string(negatives) + " pixels below zero");

  cam.stop_stream();
  std::printf("\n%s (%d failure(s))\n", failures == 0 ? "GUI CLIENT CHECK PASSED" : "GUI CLIENT CHECK FAILED",
              failures);
  return failures == 0 ? 0 : 1;
}
