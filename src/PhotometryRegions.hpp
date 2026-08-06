#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "../lib/implot/implot.h"

// The largest quantity of photometry regions. It agrees with kMaxPhotRegions in the camera server.
inline constexpr int kMaxPhotRegions = 10;

// One photometry region in image pixel coordinates: x to the right, y down, row 0 at the top.
// Both corners are inside the region. This is the convention that goes over Tango.
struct PhotRegion {
  unsigned int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
};

// ImPlot draws the heatmap with y up and the origin at the lower left. The plot y axis runs against the image row
// index. The two functions below hold that flip.
namespace photometry_regions {

// Bring an already rounded index into [0, limit].
inline unsigned int clamp_index(double value, unsigned int limit) {
  if (value < 0.0) return 0;
  if (value > (double)limit) return limit;
  return (unsigned int)value;
}

inline PhotRegion to_image_coords(const ImPlotRect &rect, unsigned int width, unsigned int height) {
  if (width == 0 || height == 0) return PhotRegion{};

  const double x_low = std::min(rect.X.Min, rect.X.Max);
  const double x_high = std::max(rect.X.Min, rect.X.Max);
  const double y_low = std::min(rect.Y.Min, rect.Y.Max);
  const double y_high = std::max(rect.Y.Min, rect.Y.Max);

  // The pixel x covers the plot range [x, x + 1). A region of the pixels x0 to x1 spans [x0, x1 + 1).
  // The near edge takes floor and the far edge takes ceil.
  const double x0 = std::floor(x_low);
  const double x1 = std::ceil(x_high) - 1.0;
  // The row r covers the plot range [height - 1 - r, height - r).
  const double y0 = (double)height - std::ceil(y_high);
  const double y1 = (double)height - 1.0 - std::floor(y_low);

  PhotRegion region;
  region.x0 = clamp_index(x0, width - 1);
  region.y0 = clamp_index(y0, height - 1);
  // A rectangle thinner than one pixel keeps its far corner at the near one.
  region.x1 = std::max(clamp_index(x1, width - 1), region.x0);
  region.y1 = std::max(clamp_index(y1, height - 1), region.y0);
  return region;
}

inline ImPlotRect to_plot_coords(const PhotRegion &region, unsigned int width, unsigned int height) {
  if (width == 0 || height == 0) return ImPlotRect();

  // The inverse of to_image_coords. The far edge of the last pixel is one unit past its index.
  ImPlotRect rect;
  rect.X.Min = (double)region.x0;
  rect.X.Max = (double)region.x1 + 1.0;
  rect.Y.Min = (double)height - 1.0 - (double)region.y1;
  rect.Y.Max = (double)height - (double)region.y0;
  return rect;
}

// The Tango PhotRegions attribute is one flat spectrum of four elements for each region.
// The quantity of active regions is the length divided by four. The element type is Tango::DevULong.
inline std::vector<uint32_t> flatten(const std::vector<PhotRegion> &regions) {
  std::vector<uint32_t> flat;
  flat.reserve(regions.size() * 4);
  for (const auto &region : regions) {
    flat.push_back(region.x0);
    flat.push_back(region.y0);
    flat.push_back(region.x1);
    flat.push_back(region.y1);
  }
  return flat;
}

inline std::vector<PhotRegion> unflatten(const std::vector<uint32_t> &flat) {
  std::vector<PhotRegion> regions;
  regions.reserve(flat.size() / 4);
  for (size_t i = 0; i + 3 < flat.size(); i += 4) {
    regions.push_back(PhotRegion{(unsigned int)flat[i], (unsigned int)flat[i + 1], (unsigned int)flat[i + 2],
                                 (unsigned int)flat[i + 3]});
  }
  return regions;
}

}  // namespace photometry_regions
