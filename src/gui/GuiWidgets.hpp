#pragma once

#include "data/ScrollingBufferT.hpp"
#include "imgui.h"
#include "lib/implot/implot.h"

// Widgets that more than one panel uses.
namespace gui {

inline void Status(const char *label, bool up) {
  const ImVec4 colour = up ? ImVec4(0.0f, 1.0f, 0.0f, 1.0f) : ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
  ImGui::TextColored(colour, "%s", label);
}

// The axes of a plot whose x axis shows the time from now - history_length to now.
inline void SetupTimePlot(double now, float history_length) {
  static ImPlotAxisFlags xflags = ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoTickLabels;
  static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
  ImPlot::SetupAxes(nullptr, nullptr, xflags, yflags);
  ImPlot::SetupAxisLimits(ImAxis_X1, now - history_length, now, ImGuiCond_Always);
  ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
  ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
}

inline void PlotSeries(const char *label, const ScrollingBufferT<double, double> &buffer, int colour, float thickness) {
  ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(colour), thickness * ImGui::GetIO().FontGlobalScale);
  ImPlot::PlotLine(label, &buffer.Data[0].time, &buffer.Data[0].value, buffer.Data.size(), 0, buffer.Offset,
                   2 * sizeof(double));
}

}  // namespace gui
