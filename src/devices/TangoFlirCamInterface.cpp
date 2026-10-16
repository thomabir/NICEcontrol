#include "devices/TangoFlirCamInterface.hpp"

#include <chrono>
#include <iostream>
#include <thread>

void TangoFlirCamInterface::start_stream() { run_command("StartStream"); }

void TangoFlirCamInterface::stop_stream() { run_command("StopFrames"); }

unsigned int TangoFlirCamInterface::get_width() { return static_cast<unsigned int>(read_attribute<ulong>("Width")); }

void TangoFlirCamInterface::write_width(unsigned int width) {
  run_command("StopFrames");
  write_attribute<unsigned long>("Width", width);
  run_command("StartStream");
}

unsigned int TangoFlirCamInterface::get_height() { return static_cast<unsigned int>(read_attribute<ulong>("Height")); }

void TangoFlirCamInterface::write_height(unsigned int height) {
  run_command("StopFrames");
  write_attribute<unsigned long>("Height", height);
  run_command("StartStream");
}

double TangoFlirCamInterface::read_framerate() { return read_attribute<double>("Framerate"); }

void TangoFlirCamInterface::write_framerate(double framerate) {
  run_command("StopFrames");
  write_attribute<double>("Framerate", framerate);
  run_command("StartStream");
}

double TangoFlirCamInterface::read_integration_time() { return read_attribute<double>("IntTime"); }

void TangoFlirCamInterface::write_integration_time(double integration_time) {
  run_command("StopFrames");
  write_attribute<double>("IntTime", integration_time);
  run_command("StartStream");
}

std::string TangoFlirCamInterface::get_sync_source() { return read_attribute<std::string>("FrameSyncSource"); }

void TangoFlirCamInterface::write_sync_source(const std::string &source) {
  run_command("StopFrames");
  write_attribute("FrameSyncSource", source);
  run_command("StartStream");
}

std::string TangoFlirCamInterface::get_sync_mode() { return read_attribute<std::string>("FrameSyncMode"); }

void TangoFlirCamInterface::write_sync_mode(const std::string &mode) {
  run_command("StopFrames");
  write_attribute("FrameSyncMode", mode);
  run_command("StartStream");
}

std::string TangoFlirCamInterface::get_sync_polarity() { return read_attribute<std::string>("FrameSyncPolarity"); }

void TangoFlirCamInterface::write_sync_polarity(const std::string &polarity) {
  run_command("StopFrames");
  write_attribute("FrameSyncPolarity", polarity);
  run_command("StartStream");
}

double TangoFlirCamInterface::get_sync_delay_us() { return read_attribute<double>("FrameSyncDelay"); }

void TangoFlirCamInterface::write_sync_delay_us(double delay_us) {
  run_command("StopFrames");
  write_attribute<double>("FrameSyncDelay", delay_us);
  run_command("StartStream");
}

std::string TangoFlirCamInterface::get_filename() { return read_attribute<std::string>("Filename"); }

void TangoFlirCamInterface::set_filename(std::string &filename) {
  run_command("StopFrames");
  write_attribute("Filename", filename);
  run_command("StartStream");
}

void TangoFlirCamInterface::start_recording(unsigned int n_frames) {
  run_command("StopFrames");
  ping_device();  // wait for reply before starting recording
  run_command<unsigned int>("RecordFrames", n_frames);
  ping_device();  // wait for reply before starting stream
  run_command("StartStream");
}

void TangoFlirCamInterface::start_recording_background(unsigned int n_frames) {
  run_command("StopFrames");
  // wait 100 ms
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  run_command<unsigned int>("CalcBackground", n_frames);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  run_command("StartStream");
}

Image<int> TangoFlirCamInterface::get_image() {
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

Image<int> TangoFlirCamInterface::get_image_bg_sub() {
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

void TangoFlirCamInterface::set_regions(const std::vector<PhotRegion> &regions) {
  write_spectrum_attribute<Tango::DevULong>("PhotRegions", photometry_regions::flatten(regions));
}

std::vector<PhotRegion> TangoFlirCamInterface::get_regions() {
  return photometry_regions::unflatten(read_spectrum_attribute<Tango::DevULong>("PhotRegions"));
}

bool TangoFlirCamInterface::get_phot_subtract_background() { return read_attribute<bool>("PhotSubtractBackground"); }

void TangoFlirCamInterface::set_phot_subtract_background(bool on) {
  write_attribute<bool>("PhotSubtractBackground", on);
}

PhotBatch TangoFlirCamInterface::get_phot_since(uint64_t last_frame_id) {
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
    std::cerr << "FlirCam: GetPhotSince reply has " << reply.size() << " elements, but " << n_records << " records of "
              << stride << " elements need " << (2 + n_records * stride) << ". Reply dropped." << std::endl;
    return batch;
  }

  batch.samples.reserve(n_records);
  for (size_t i = 0; i < n_records; i++) {
    const double *record = &reply[2 + i * stride];
    PhotSample sample;
    sample.frame_id = static_cast<uint64_t>(record[0]);
    sample.t_mono = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(static_cast<int64_t>(record[1])));
    sample.t_cam_ns = record[2];
    for (size_t r = 0; r < batch.n_regions; r++) {
      sample.values[r] = record[3 + r];
    }
    batch.samples.push_back(sample);
  }
  return batch;
}
