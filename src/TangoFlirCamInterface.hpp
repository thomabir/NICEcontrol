#pragma once

#include <array>
#include <cmath>
#include <ctime>
#include <numeric>
#include <tuple>
#include <variant>
#include <vector>

#include "Image.hpp"
#include "TangoGenericInterface.hpp"

class TangoFlirCamInterface : public TangoGenericInterface {
 public:
  TangoFlirCamInterface() : TangoGenericInterface("detectors/flir/1") {}

  void start_stream() { run_command("StartStream"); }

  void stop_stream() { run_command("StopFrames"); }

  int get_width() { return read_attribute<ulong>("Width"); }

  void write_width(unsigned int width) {
    run_command("StopFrames");
    write_attribute<unsigned long>("Width", width);
    run_command("StartStream");
  }

  int get_height() { return read_attribute<ulong>("Height"); }

  void write_height(unsigned int height) {
    run_command("StopFrames");
    write_attribute<unsigned long>("Height", height);
    run_command("StartStream");
  }

  double read_framerate() { return read_attribute<double>("Framerate"); }

  void write_framerate(double framerate) {
    run_command("StopFrames");
    write_attribute<double>("Framerate", framerate);
    run_command("StartStream");
  }

  double read_integration_time() { return read_attribute<double>("IntTime"); }

  void write_integration_time(double integration_time) { write_attribute<double>("IntTime", integration_time); }

  std::string get_filename() { return read_attribute<std::string>("Filename"); }

  void set_filename(std::string &filename) { write_attribute("Filename", filename); }

  void start_recording(unsigned int n_frames) { run_command<unsigned int>("RecordFrames", n_frames); }
  void start_recording_background(unsigned int n_frames) { run_command<unsigned int>("CalcBackground", n_frames); }

  Image<int> get_image() {
    Image<int> image;
    // Use the new method to get the image data and dimensions in one call
    Image<unsigned short> image_short = read_image<unsigned short>("Image");

    // convert to signed int
    image.data.resize(image_short.data.size());
    for (size_t i = 0; i < image_short.data.size(); ++i) {
      image.data[i] = static_cast<int>(image_short.data[i]);
    }
    image.width = image_short.width;
    image.height = image_short.height;

    return image;
  }

  std::vector<unsigned short> get_background() { return read_attribute<std::vector<unsigned short>>("Background"); }
};
