#pragma once

#include <cmath>
#include <numeric>
#include <vector>

#include "algorithms/FftCalculator.hpp"
#include "gui/GuiWidgets.hpp"
#include "gui/PlotHistory.hpp"
#include "imgui.h"
#include "lib/implot/implot.h"

// The signals that the PLC sends over EtherCAT: the delay line, the OPD with its spectrum, and the QPDs.
class EthercatPanel {
 public:
  explicit EthercatPanel(const PlotHistory &plot_history) : history(plot_history) {}

  void Render() {
    const float font_scale = ImGui::GetIO().FontGlobalScale;

    if (ImGui::TreeNode("Delay line##EtherCAT")) {
      static float history_length = 10.0f;
      ImGui::SliderFloat("History length##DL", &history_length, 0.1f, 10.0f, "%.2f s", ImGuiSliderFlags_Logarithmic);
      if (ImPlot::BeginPlot("##EtherCAT Delay Line", ImVec2(-1, 200 * font_scale))) {
        gui::SetupTimePlot(history.plc_time, history_length);
        gui::PlotSeries("DL position (um)", history.dl_pos_buffer, 0, 3.0f);
        gui::PlotSeries("DL command (um)", history.dl_cmd_buffer, 1, 3.0f);
        ImPlot::EndPlot();
      }
      ImGui::TreePop();
    }

    if (ImGui::TreeNode("OPD##EtherCAT")) {
      static float history_length = 10.0f;
      ImGui::SliderFloat("History length##OPD", &history_length, 0.1f, 10.0f, "%.2f s", ImGuiSliderFlags_Logarithmic);
      if (ImPlot::BeginPlot("##EtherCAT OPD", ImVec2(-1, 200 * font_scale))) {
        gui::SetupTimePlot(history.plc_time, history_length);
        gui::PlotSeries("OPD (um)", history.opd_buffer, 1, 1.0f);
        ImPlot::EndPlot();
      }

      const int n_points = 1000;
      static double mean = 0.0;    // um
      static double stddev = 0.0;  // um
      if (history.opd_buffer.Data.size() > n_points + 2) {
        const auto last = history.opd_buffer.GetLastN(n_points);
        std::vector<double> values;
        values.reserve(n_points);
        for (const auto &m : last) {
          values.push_back(m.value);
        }
        // Two passes, because the mean is large against the deviation and one pass loses the deviation.
        const double count = static_cast<double>(values.size());
        mean = std::accumulate(values.begin(), values.end(), 0.0) / count;
        double square_sum = 0.0;
        for (const double value : values) {
          square_sum += (value - mean) * (value - mean);
        }
        stddev = std::sqrt(square_sum / count);
      }
      ImGui::Text("Last %d samples: Mean: %.4f um, Std: %.4f nm", n_points, mean, stddev * 1e3);
      ImGui::TreePop();
    }

    if (ImGui::TreeNode("OPD FFT##EtherCAT")) {
      const static double fs = 1 / 150e-6;   // Hz, the PLC samples every 150 us
      const static int fft_size = 1024 * 8;  // samples
      static double fft_power[fft_size / 2];
      static double fft_freq[fft_size / 2];
      static FFT_calculator<double, double> fft(fft_size, fs, &history.opd_buffer, fft_power, fft_freq);

      fft.calculate();

      static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
      if (ImPlot::BeginPlot("##FFT", ImVec2(-1, 400 * font_scale))) {
        ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
        ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
        ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_None, yflags);
        ImPlot::SetupAxisLimits(ImAxis_X1, 1, 3333);
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(1), 3);
        ImPlot::PlotLine("OPD PSD (nm/sqrtHz)", &fft_freq[0], &fft_power[0], fft_size / 2);
        ImPlot::EndPlot();
      }
      ImGui::TreePop();
    }

    if (ImGui::TreeNode("QPD##EtherCAT")) {
      static const char *labels[12] = {"QPD1 X1", "QPD1 Y1", "QPD1 I1", "QPD1 X2", "QPD1 Y2", "QPD1 I2",
                                       "QPD2 X1", "QPD2 Y1", "QPD2 I1", "QPD2 X2", "QPD2 Y2", "QPD2 I2"};
      static float history_length = 10.0f;
      ImGui::SliderFloat("History length##QPD", &history_length, 0.1f, 10.0f, "%.2f s", ImGuiSliderFlags_Logarithmic);
      if (ImPlot::BeginPlot("##EtherCAT QPD", ImVec2(-1, 200 * font_scale))) {
        gui::SetupTimePlot(history.plc_time, history_length);
        for (int i = 0; i < 12; i++) {
          gui::PlotSeries(labels[i], history.qpd_buffers[i], i, 1.0f);
        }
        ImPlot::EndPlot();
      }
      ImGui::TreePop();
    }
  }

 private:
  const PlotHistory &history;
};
