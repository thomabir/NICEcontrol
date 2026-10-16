#pragma once

#include "gui/PlotHistory.hpp"
#include "imgui.h"
#include "lib/implot/implot.h"

// The channels of the ADC, as a function of the sample counter.
class AdcPanel {
 public:
  explicit AdcPanel(const PlotHistory &plot_history) : history(plot_history) {}

  void Render() {
    static float history_length = 1000.f;
    ImGui::SliderFloat("ADC History", &history_length, 200, 70000, "%.0f samples", ImGuiSliderFlags_Logarithmic);

    static const char *labels[16] = {
        "QPD1 UP",        "QPD1 LEFT", "QPD1 RIGHT",     "QPD1 DOWN", "QPD1 sum", "OPD ref",
        "Pos ref (QPD1)", "OPD back",  "Pos ref (QPD2)", "NC3",       "NC4",      "NC5",
        "QPD2 UP",        "QPD2 LEFT", "QPD2 RIGHT",     "QPD2 DOWN"};

    static ImPlotAxisFlags xflags = ImPlotAxisFlags_NoTickLabels;
    static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
    if (ImPlot::BeginPlot("##ADC", ImVec2(-1, 400 * ImGui::GetIO().FontGlobalScale))) {
      ImPlot::SetupAxes(nullptr, nullptr, xflags, yflags);
      ImPlot::SetupAxisLimits(ImAxis_X1, static_cast<double>(history.adc_time) - history_length, history.adc_time,
                              ImGuiCond_Always);
      ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
      ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
      for (int i = 0; i < 16; i++) {
        const ScrollingBufferT<int, int> &buffer = history.adc_buffers[i];
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(i), 2);
        ImPlot::PlotLine(labels[i], &buffer.Data[0].time, &buffer.Data[0].value, buffer.Data.size(), 0, buffer.Offset,
                         2 * sizeof(int));
      }
      ImPlot::EndPlot();
    }
  }

 private:
  const PlotHistory &history;
};
