#pragma once

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "core/Commands.hpp"
#include "core/Whiteboard.hpp"
#include "data/PhotometryRegions.hpp"
#include "gui/PlotHistory.hpp"
#include "imgui.h"
#include "lib/implot/implot.h"

// The FLIR camera, in three parts: Camera settings, Image, Photometry. The camera application owns the device and
// measures the photometry. This panel draws what it reports and asks it for changes.
class CameraPanel {
 public:
  CameraPanel(CommandBox &command_box, Whiteboard &whiteboard, const PlotHistory &plot_history)
      : commands(command_box), wb(whiteboard), history(plot_history) {
    phot_intensity_of_one.resize(kMaxPhotRegions, 1.0f);
  }

  void Render(const Snapshot &snap) {
    static ImVec2 window_size(600, 800);
    ImGui::SetNextWindowSize(window_size, ImGuiCond_FirstUseEver);
    ImGui::Begin("FlirCamWindow", nullptr, 0);

    if (!snap.camera.connected) {
      if (ImGui::Button("Connect")) {
        commands.edit([](Commands &c) { c.camera.connect = true; });
      }
      ImGui::SameLine();
      ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "Disconnected");
      seeded = false;
      ImGui::End();
      return;
    }

    if (ImGui::Button("Disconnect")) {
      commands.edit([](Commands &c) { c.camera.connect = false; });
    }
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Connected");

    // The camera reports the truth, so the panel starts from what the core last read out of it.
    if (!seeded && !snap.camera.regions.empty()) {
      seeded = true;
      region_count = static_cast<int>(snap.camera.regions.size());
      sent_regions = snap.camera.regions;
      rects_from_device = true;
      subtract_background = snap.camera.subtract_background;
    }

    if (ImGui::CollapsingHeader("Camera settings")) {
      Settings(snap);
    }
    if (ImGui::CollapsingHeader("Image", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImageView();
    }
    if (ImGui::CollapsingHeader("Photometry", ImGuiTreeNodeFlags_DefaultOpen)) {
      Photometry(snap);
    }

    PushRegionsIfChanged();
    ImGui::End();
  }

 private:
  CommandBox &commands;
  Whiteboard &wb;
  const PlotHistory &history;

  // The rectangles on screen become the regions that the camera measures.
  bool seeded = false;
  int region_count = 1;
  std::vector<ImPlotRect> rects;
  bool rects_from_device = false;
  bool subtract_background = false;
  std::vector<PhotRegion> sent_regions;
  std::shared_ptr<const Measurement<Image<int>>> frame;

  // Local display scaling of the photometry. It divides what the panel shows and it does not reach the camera.
  std::vector<float> phot_intensity_of_one;
  bool phot_apply_intensity_of_one = false;
  bool phot_apply_nd_filter = false;
  float phot_nd_filter_factor = 1.0f;

  // One entry of a camera enumeration feature: the entry name that the camera uses, and the label on screen.
  struct Choice {
    const char *entry;
    const char *label;
  };

  static constexpr Choice kSyncSources[] = {{"Internal", "Internal"}, {"External", "External"}};
  static constexpr Choice kSyncModes[] = {{"Integration", "FSSI"}, {"Readout", "FSSR"}};
  static constexpr Choice kSyncPolarities[] = {{"ActiveLow", "Active low"}, {"ActiveHigh", "Active high"}};

  void Settings(const Snapshot &snap) {
    ImGui::Text("Commands:");
    ImGui::SameLine();
    for (const std::string &command : snap.camera.device_commands) {
      if (ImGui::Button((command + "##FlirCommand").c_str())) {
        commands.edit([&](Commands &c) {
          c.camera.device_command = command;
          c.camera.device_command_count++;
        });
      }
      ImGui::SameLine();
    }
    ImGui::NewLine();

    static char filename[128] = "";
    ImGui::Text("Filename: %s", snap.camera.filename.c_str());
    ImGui::SameLine();
    ImGui::SetNextItemWidth(400);
    ImGui::InputText("##Filename", filename, sizeof(filename));
    ImGui::SameLine();
    if (ImGui::Button("Set##Filename")) {
      commands.edit([](Commands &c) { c.camera.filename = std::string(filename); });
    }

    static unsigned int n_frames = 1;
    ImGui::Text("Frames:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    ImGui::InputScalar("##NFrames", ImGuiDataType_U32, &n_frames, NULL, NULL, "%d");
    ImGui::SameLine();
    if (ImGui::Button("Record##NFrames")) {
      commands.edit([](Commands &c) {
        c.camera.record_frames = n_frames;
        c.camera.record_count++;
      });
    }

    static unsigned int n_frames_bg = 1;
    ImGui::SameLine();
    ImGui::Text("BG frames:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    ImGui::InputScalar("##NFramesBG", ImGuiDataType_U32, &n_frames_bg, NULL, NULL, "%d");
    ImGui::SameLine();
    if (ImGui::Button("Record BG##NFrames")) {
      commands.edit([](Commands &c) {
        c.camera.background_frames = n_frames_bg;
        c.camera.background_count++;
      });
    }

    SettingRow<double>("Framerate", "%.4f Hz", snap.camera.framerate, ImGuiDataType_Double,
                       [](Commands &c, double v) { c.camera.framerate = v; });
    SettingRow<double>("Integration time", "%.4f ms", snap.camera.integration_time_ms, ImGuiDataType_Double,
                       [](Commands &c, double v) { c.camera.integration_time_ms = v; });
    SettingRow<unsigned int>("Width", "%d px", snap.camera.width, ImGuiDataType_U32,
                             [](Commands &c, unsigned int v) { c.camera.width = v; });
    SettingRow<unsigned int>("Height", "%d px", snap.camera.height, ImGuiDataType_U32,
                             [](Commands &c, unsigned int v) { c.camera.height = v; });

    ImGui::SeparatorText("Frame sync");
    ChoiceRow("Sync source", snap.camera.sync_source, kSyncSources,
              [](Commands &c, const std::string &v) { c.camera.sync_source = v; });
    ChoiceRow("Sync mode", snap.camera.sync_mode, kSyncModes,
              [](Commands &c, const std::string &v) { c.camera.sync_mode = v; });
    ChoiceRow("Sync polarity", snap.camera.sync_polarity, kSyncPolarities,
              [](Commands &c, const std::string &v) { c.camera.sync_polarity = v; });
    SettingRow<double>("Sync delay", "%.3f us", snap.camera.sync_delay_us, ImGuiDataType_Double,
                       [](Commands &c, double v) { c.camera.sync_delay_us = v; });
  }

  // Shows the entry that the camera reports as the selected radio button, and sends an entry when the user selects it.
  template <size_t N, typename Apply>
  void ChoiceRow(const char *label, const std::string &reported, const Choice (&choices)[N], Apply apply) {
    ImGui::Text("%s: ", label);
    ImGui::PushID(label);
    for (const Choice &choice : choices) {
      ImGui::SameLine();
      if (ImGui::RadioButton(choice.label, reported == choice.entry) && reported != choice.entry) {
        const std::string value = choice.entry;
        commands.edit([&](Commands &c) { apply(c, value); });
      }
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(camera: %s)", reported.c_str());
    ImGui::PopID();
  }

  // Shows the value that the camera reports, and sends a new one when the user asks for it.
  template <typename T, typename Apply>
  void SettingRow(const char *label, const char *format, T reported, ImGuiDataType type, Apply apply) {
    ImGui::Text("%s: ", label);
    ImGui::SameLine();
    ImGui::Text(format, reported);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120);
    // One input value per row. Rows of the same type share the template instance, thus the label selects the value.
    static std::map<std::string, T> wanted_by_label;
    T &wanted = wanted_by_label[label];
    ImGui::PushID(label);
    ImGui::InputScalar("##Wanted", type, &wanted, NULL, NULL, format);
    ImGui::SameLine();
    if (ImGui::Button("Set")) {
      const T value = wanted;
      commands.edit([&](Commands &c) { apply(c, value); });
    }
    ImGui::PopID();
  }

  void ImageView() {
    const float font_scale = ImGui::GetIO().FontGlobalScale;

    static int image_product = 0;
    ImGui::Text("Show:");
    ImGui::SameLine();
    bool product_changed = ImGui::RadioButton("Raw", &image_product, 0);
    ImGui::SameLine();
    product_changed |= ImGui::RadioButton("Background subtracted", &image_product, 1);
    if (product_changed) {
      commands.edit([](Commands &c) { c.camera.image_product = image_product; });
    }

    frame = wb.camera_image.load();

    static ImPlotColormap map = ImPlotColormap_Viridis;
    if (ImPlot::ColormapButton(ImPlot::GetColormapName(map), ImVec2(225, 0), map)) {
      map = (map + 1) % ImPlot::GetColormapCount();
      ImPlot::BustColorCache("##Heatmap");
    }

    ImPlot::PushColormap(map);
    ImGui::SameLine();
    static bool autoscale_colormap = true;
    ImGui::Checkbox("Autoscale", &autoscale_colormap);

    if (!HasImage()) {
      ImPlot::PopColormap();
      ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.0f, 1.0f), "No image. Is the camera streaming?");
      return;
    }

    const Image<int> &camera_image = frame->value;
    const unsigned int width = camera_image.width;
    const unsigned int height = camera_image.height;
    const int *values = camera_image.data.data();

    const float aspect_ratio = float(width) / float(height);
    const float plot_width = ImGui::GetContentRegionAvail().x - 80 * font_scale;
    const float plot_height = plot_width / aspect_ratio;

    static int scale_min = 0;
    static int scale_max = 16383;
    const int scale_max_default = 16383;
    const int scale_min_default = -16384;

    ImGui::SetNextItemWidth(400);
    ImGui::SliderScalar("Min", ImGuiDataType_S32, &scale_min, &scale_min_default, &scale_max_default, "%d");
    ImGui::SetNextItemWidth(400);
    ImGui::SameLine();
    ImGui::SliderScalar("Max", ImGuiDataType_S32, &scale_max, &scale_min_default, &scale_max_default, "%d");

    if (autoscale_colormap) {
      scale_min = *std::min_element(camera_image.data.begin(), camera_image.data.end());
      scale_max = *std::max_element(camera_image.data.begin(), camera_image.data.end());
    }
    if (scale_min >= scale_max) {
      scale_min = scale_max - 1;
    }

    if (rects.size() != static_cast<size_t>(region_count)) {
      rects.resize(region_count);
    }

    // A first connect puts the regions of the camera on screen.
    if (rects_from_device) {
      rects_from_device = false;
      for (size_t i = 0; i < rects.size() && i < sent_regions.size(); i++) {
        rects[i] = photometry_regions::to_plot_coords(sent_regions[i], width, height);
      }
    }

    if (ImPlot::BeginPlot("##Heatmap", ImVec2(plot_width, plot_height), ImPlotFlags_NoMouseText)) {
      ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_NoDecorations, ImPlotAxisFlags_NoDecorations);
      ImPlot::PlotHeatmap("heat", values, static_cast<int>(height), static_cast<int>(width), scale_min, scale_max,
                          nullptr, ImPlotPoint(0, 0), ImPlotPoint(width, height));
      for (int i = 0; i < region_count; i++) {
        if (rects[i].X.Min == rects[i].X.Max && rects[i].Y.Min == rects[i].Y.Max) {
          const double x_offset = (width * 0.05) * i;
          const double y_offset = (height * 0.05) * i;
          rects[i] = {0 + x_offset, double(width) / 2 + x_offset, 0 + y_offset, double(height) / 2 + y_offset};
        }
        const ImVec4 rect_color = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
        ImPlot::DragRect(i, &rects[i].X.Min, &rects[i].Y.Min, &rects[i].X.Max, &rects[i].Y.Max, rect_color,
                         ImPlotDragToolFlags_None);
        ImVec2 pixel_min = ImPlot::PlotToPixels(ImPlotPoint(rects[i].X.Min, rects[i].Y.Min));
        ImVec2 pixel_max = ImPlot::PlotToPixels(ImPlotPoint(rects[i].X.Max, rects[i].Y.Max));
        if (pixel_min.x > pixel_max.x) {
          std::swap(pixel_min.x, pixel_max.x);
        }
        if (pixel_min.y > pixel_max.y) {
          std::swap(pixel_min.y, pixel_max.y);
        }
        ImPlot::PushPlotClipRect();
        ImPlot::GetPlotDrawList()->AddRect(pixel_min, pixel_max, ImGui::ColorConvertFloat4ToU32(rect_color), 0.0f, 0,
                                           2.5f);
        ImPlot::PopPlotClipRect();
        const std::string label = std::to_string(i + 1);
        ImGui::SetWindowFontScale(2.0f);
        ImPlot::PlotText(label.c_str(), rects[i].X.Min, rects[i].Y.Max, ImVec2(24.0f, 24.0f));
        ImGui::SetWindowFontScale(1.0f);
      }
      ImPlot::EndPlot();
    }
    ImGui::SameLine();
    ImPlot::ColormapScale("##HeatScale", scale_min, scale_max, ImVec2(80 * font_scale, plot_height));
    ImPlot::PopColormap();
  }

  // The core has given an image, and that image has pixels.
  bool HasImage() const {
    return frame && !frame->value.data.empty() && frame->value.width != 0 && frame->value.height != 0;
  }

  // The regions follow the rectangles as they move.
  void PushRegionsIfChanged() {
    if (!HasImage()) {
      return;
    }

    const Image<int> &camera_image = frame->value;
    std::vector<PhotRegion> regions;
    regions.reserve(region_count);
    for (int i = 0; i < region_count && i < static_cast<int>(rects.size()); i++) {
      regions.push_back(photometry_regions::to_image_coords(rects[i], camera_image.width, camera_image.height));
    }

    bool same = regions.size() == sent_regions.size();
    for (size_t i = 0; i < regions.size() && same; i++) {
      same = regions[i].x0 == sent_regions[i].x0 && regions[i].y0 == sent_regions[i].y0 &&
             regions[i].x1 == sent_regions[i].x1 && regions[i].y1 == sent_regions[i].y1;
    }
    if (same) {
      return;
    }

    commands.edit([&](Commands &c) { c.camera.regions = regions; });
    sent_regions = regions;
  }

  // One photometry series, and the scaling that the panel draws it with.
  struct PhotSeries {
    const ScrollingBufferT<double, double> *buffer;
    double scale;
  };

  // The buffers hold what the camera measured, thus the scaling happens here and the history stays untouched.
  static ImPlotPoint PhotSeriesPoint(int index, void *user_data) {
    const PhotSeries &series = *static_cast<const PhotSeries *>(user_data);
    const ImVector<PlotPoint<double, double>> &data = series.buffer->Data;
    const PlotPoint<double, double> &point = data[(series.buffer->Offset + index) % data.size()];
    return ImPlotPoint(point.time, point.value * series.scale);
  }

  // What the y axis of the photometry plot carries. The scaling is a division, thus the label names the divisor.
  const char *PhotAxisLabel() const {
    if (phot_apply_intensity_of_one && phot_apply_nd_filter) {
      return "Counts / (I_0 ND)";
    }
    if (phot_apply_intensity_of_one) {
      return "Counts / I_0";
    }
    if (phot_apply_nd_filter) {
      return "Counts / ND";
    }
    return "Counts";
  }

  // What one region of the photometry is divided by. A factor of zero divides by one instead.
  double PhotScale(size_t region) const {
    double scale = 1.0;
    if (phot_apply_intensity_of_one && region < phot_intensity_of_one.size() && phot_intensity_of_one[region] != 0.0f) {
      scale /= phot_intensity_of_one[region];
    }
    if (phot_apply_nd_filter && phot_nd_filter_factor != 0.0f) {
      scale /= phot_nd_filter_factor;
    }
    return scale;
  }

  void Photometry(const Snapshot &snap) {
    const float font_scale = ImGui::GetIO().FontGlobalScale;
    // The width follows the font, because a stepper of a fixed width does not hold its buttons at a large scale and
    // the row then breaks into two.
    const float input_width = 120 * font_scale;

    if (ImGui::Checkbox("Subtract background", &subtract_background)) {
      const bool on = subtract_background;
      commands.edit([on](Commands &c) { c.camera.subtract_background = on; });
    }
    ImGui::SameLine();
    // What the camera reports back. It follows the box after one round of the settings, so a write that does not
    // arrive is visible here.
    ImGui::TextDisabled("(camera: %s)", snap.camera.subtract_background ? "on" : "off");
    ImGui::SameLine();
    ImGui::Text("Regions:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(input_width);
    if (ImGui::InputInt("##RegionCount", &region_count, 1, 1)) {
      region_count = std::max(0, std::min(region_count, kMaxPhotRegions));
    }

    // The scaling reaches the rows and the plot.
    ImGui::Text("Normalise:");
    ImGui::SameLine();
    ImGui::Checkbox("I_0##Apply one", &phot_apply_intensity_of_one);
    ImGui::SameLine();
    ImGui::Checkbox("ND##Apply ND", &phot_apply_nd_filter);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(input_width);
    ImGui::InputFloat("##ND filter factor", &phot_nd_filter_factor, 0.01f, 0.1f, "%.2f");

    const size_t shown = std::min(static_cast<size_t>(region_count), snap.camera.n_regions);
    for (size_t i = 0; i < shown; i++) {
      ImGui::Text("%zu: %.3e", i + 1, snap.camera.values[i] * PhotScale(i));
      ImGui::SameLine();
      ImGui::Text("I_0:");
      ImGui::SameLine();
      ImGui::SetNextItemWidth(input_width);
      ImGui::InputFloat(("##I_0_" + std::to_string(i)).c_str(), &phot_intensity_of_one[i], 0.01f, 0.1f, "%.2f");
    }

    static float history_length = 10.f;
    ImGui::SliderFloat("History", &history_length, 1, 20, "%.5f s", ImGuiSliderFlags_Logarithmic);
    if (ImPlot::BeginPlot("Sum Intensity", ImVec2(-1, 400 * font_scale))) {
      static ImPlotAxisFlags yflags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit;
      ImPlot::SetupAxes(nullptr, PhotAxisLabel(), ImPlotAxisFlags_AutoFit, yflags);
      ImPlot::SetupAxisLimits(ImAxis_X1, history.phot_time - history_length, history.phot_time, ImGuiCond_Always);
      ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
      ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
      for (int i = 0; i < region_count; i++) {
        PhotSeries series{&history.phot_buffers[i], PhotScale(static_cast<size_t>(i))};
        ImPlot::SetNextLineStyle(ImPlot::GetColormapColor(i % ImPlot::GetColormapCount()), 2.0f * font_scale);
        ImPlot::PlotLineG(std::to_string(i + 1).c_str(), PhotSeriesPoint, &series, series.buffer->Data.size());
      }
      ImPlot::EndPlot();
    }
  }
};
