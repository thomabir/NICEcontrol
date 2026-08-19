#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <iostream>
#include <numeric>
#include <thread>
#include <tuple>
#include <variant>
#include <vector>

#include "data/Image.hpp"
#include "data/PhotometryRegions.hpp"
#include "devices/TangoGenericInterface.hpp"

// The photometry of one frame, as GetPhotSince gives it.
struct PhotSample {
  uint64_t frame_id = 0;
  int64_t t_PC_ns = 0;    // the monotonic PC clock, read by the server when the frame arrived
  double t_cam_ns = 0.0;  // the clock of the camera, as the frame carries it
  std::array<double, kMaxPhotRegions> values{};
};

// The reply of one GetPhotSince call.
struct PhotBatch {
  size_t n_regions = 0;
  std::vector<PhotSample> samples;
};

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

  void write_integration_time(double integration_time) {
    run_command("StopFrames");
    write_attribute<double>("IntTime", integration_time);
    run_command("StartStream");
  }

  std::string get_filename() { return read_attribute<std::string>("Filename"); }

  void set_filename(std::string &filename) {
    run_command("StopFrames");
    write_attribute("Filename", filename);
    run_command("StartStream");
  }

  void start_recording(unsigned int n_frames) {
    run_command("StopFrames");
    ping_device();  // wait for reply before starting recording
    run_command<unsigned int>("RecordFrames", n_frames);
    ping_device();  // wait for reply before starting stream
    run_command("StartStream");
  }

  void start_recording_background(unsigned int n_frames) {
    run_command("StopFrames");
    // wait 100 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    run_command<unsigned int>("CalcBackground", n_frames);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    run_command("StartStream");
  }

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

  // The camera makes this image: raw minus background. The type is signed.
  Image<int> get_image_bg_sub() {
    Image<int> image;
    Image<short> image_short = read_image<short>("ImageBgSub");

    image.data.resize(image_short.data.size());
    for (size_t i = 0; i < image_short.data.size(); ++i) {
      image.data[i] = static_cast<int>(image_short.data[i]);
    }
    image.width = image_short.width;
    image.height = image_short.height;

    return image;
  }

  std::vector<unsigned short> get_background() { return read_attribute<std::vector<unsigned short>>("Background"); }

  // A write of the regions does not interrupt the acquisition.
  void set_regions(const std::vector<PhotRegion> &regions) {
    write_spectrum_attribute<Tango::DevULong>("PhotRegions", photometry_regions::flatten(regions));
  }

  std::vector<PhotRegion> get_regions() {
    return photometry_regions::unflatten(read_spectrum_attribute<Tango::DevULong>("PhotRegions"));
  }

  bool get_phot_subtract_background() { return read_attribute<bool>("PhotSubtractBackground"); }

  void set_phot_subtract_background(bool on) { write_attribute<bool>("PhotSubtractBackground", on); }

  // Ask for every photometry sample after last_frame_id.
  // The reply is [n_regions, n_records, records...]. One record is [frame_id, t_PC_ns, t_cam_ns, v_0 .. v_(n-1)].
  PhotBatch get_phot_since(uint64_t last_frame_id) {
    PhotBatch batch;
    const auto reply = run_command_with_reply<std::vector<double>, uint64_t>("GetPhotSince", last_frame_id);
    if (reply.size() < 2) {
      return batch;
    }

    batch.n_regions = static_cast<size_t>(reply[0]);
    const size_t n_records = static_cast<size_t>(reply[1]);
    if (batch.n_regions > kMaxPhotRegions) {
      std::cerr << "FlirCam: GetPhotSince reports " << batch.n_regions << " regions, more than the maximum of "
                << kMaxPhotRegions << ". Reply dropped." << std::endl;
      batch.n_regions = 0;
      return batch;
    }

    const size_t stride = batch.n_regions + 3;
    if (reply.size() != 2 + n_records * stride) {
      std::cerr << "FlirCam: GetPhotSince reply has " << reply.size() << " elements, but " << n_records
                << " records of " << stride << " elements need " << (2 + n_records * stride) << ". Reply dropped."
                << std::endl;
      return batch;
    }

    batch.samples.reserve(n_records);
    for (size_t i = 0; i < n_records; i++) {
      const double *record = &reply[2 + i * stride];
      PhotSample sample;
      sample.frame_id = static_cast<uint64_t>(record[0]);
      sample.t_PC_ns = static_cast<int64_t>(record[1]);
      sample.t_cam_ns = record[2];
      for (size_t r = 0; r < batch.n_regions; r++) {
        sample.values[r] = record[3 + r];
      }
      batch.samples.push_back(sample);
    }
    return batch;
  }
};
