// Checks the coordinate conversion in src/PhotometryRegions.hpp.
#include <cstdio>
#include <string>

#include "../src/PhotometryRegions.hpp"

static int failures = 0;

static void expect_region(const std::string &what, const PhotRegion &got, unsigned int x0, unsigned int y0,
                          unsigned int x1, unsigned int y1) {
  const bool ok = got.x0 == x0 && got.y0 == y0 && got.x1 == x1 && got.y1 == y1;
  std::printf("%-46s got (%u,%u)-(%u,%u) want (%u,%u)-(%u,%u)  %s\n", what.c_str(), got.x0, got.y0, got.x1, got.y1, x0,
              y0, x1, y1, ok ? "OK" : "FAIL");
  if (!ok) failures++;
}

int main() {
  const unsigned int width = 640;
  const unsigned int height = 512;

  // The lowest band of the plot is the last rows of the image.
  expect_region("plot bottom band -> last image rows",
                photometry_regions::to_image_coords(ImPlotRect(0.0, 10.0, 0.0, 10.0), width, height), 0, 502, 9, 511);

  // The highest band of the plot is the first rows of the image.
  expect_region("plot top band -> first image rows",
                photometry_regions::to_image_coords(ImPlotRect(0.0, 10.0, 502.0, 512.0), width, height), 0, 0, 9, 9);

  // Corners in the wrong order give an ordered region.
  expect_region("reversed corners are put in order",
                photometry_regions::to_image_coords(ImPlotRect(100.0, 20.0, 300.0, 100.0), width, height), 20, 212, 99,
                411);

  // A rectangle thinner than one pixel still covers one pixel.
  expect_region("sub-pixel rectangle covers one pixel",
                photometry_regions::to_image_coords(ImPlotRect(4.2, 4.4, 4.2, 4.4), width, height), 4, 507, 4, 507);

  // A rectangle past the frame is cut to the frame.
  expect_region("out of frame is clamped",
                photometry_regions::to_image_coords(ImPlotRect(-50.0, 5000.0, -50.0, 5000.0), width, height), 0, 0, 639,
                511);

  // region -> plot -> region gives the region back.
  const PhotRegion cases[] = {
      {0, 0, 0, 0}, {10, 20, 30, 40}, {639, 511, 639, 511}, {5, 0, 634, 511}, {320, 256, 320, 256}};
  for (const auto &start : cases) {
    const ImPlotRect rect = photometry_regions::to_plot_coords(start, width, height);
    const PhotRegion back = photometry_regions::to_image_coords(rect, width, height);
    expect_region("round trip (" + std::to_string(start.x0) + "," + std::to_string(start.y0) + ")", back, start.x0,
                  start.y0, start.x1, start.y1);
  }

  // The flat form keeps the order x0, y0, x1, y1.
  const std::vector<PhotRegion> regions{{1, 2, 3, 4}, {5, 6, 7, 8}};
  const auto flat = photometry_regions::flatten(regions);
  const bool flat_ok = flat.size() == 8 && flat[0] == 1 && flat[1] == 2 && flat[2] == 3 && flat[3] == 4 &&
                       flat[4] == 5 && flat[5] == 6 && flat[6] == 7 && flat[7] == 8;
  std::printf("%-46s %s\n", "flatten keeps x0,y0,x1,y1 order", flat_ok ? "OK" : "FAIL");
  if (!flat_ok) failures++;

  const auto restored = photometry_regions::unflatten(flat);
  const bool unflat_ok = restored.size() == 2 && restored[1].x0 == 5 && restored[1].y1 == 8;
  std::printf("%-46s %s\n", "unflatten gives the regions back", unflat_ok ? "OK" : "FAIL");
  if (!unflat_ok) failures++;

  std::printf("\n%s (%d failure(s))\n", failures == 0 ? "REGION TESTS PASSED" : "REGION TESTS FAILED", failures);
  return failures == 0 ? 0 : 1;
}
