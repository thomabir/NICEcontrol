#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "data/Image.hpp"
#include "data/PhotSample.hpp"
#include "data/PhotometryRegions.hpp"
#include "devices/TangoGenericInterface.hpp"

class TangoFlirCamInterface : public TangoGenericInterface {
 public:
  TangoFlirCamInterface() : TangoGenericInterface("detectors/flir/1") {}

  void start_stream();
  void stop_stream();
  unsigned int get_width();
  void write_width(unsigned int width);
  unsigned int get_height();
  void write_height(unsigned int height);
  double read_framerate();
  void write_framerate(double framerate);
  double read_integration_time();
  void write_integration_time(double integration_time);

  // The frame sync settings. The server refuses these writes during an acquisition, thus each write stops the stream
  // and starts it again.
  std::string get_sync_source();
  void write_sync_source(const std::string &source);
  std::string get_sync_mode();
  void write_sync_mode(const std::string &mode);
  std::string get_sync_polarity();
  void write_sync_polarity(const std::string &polarity);
  double get_sync_delay_us();
  void write_sync_delay_us(double delay_us);

  std::string get_filename();
  void set_filename(std::string &filename);
  void start_recording(unsigned int n_frames);
  void start_recording_background(unsigned int n_frames);
  Image<int> get_image();

  // The camera makes this image: raw minus background. The type is signed.
  Image<int> get_image_bg_sub();

  // A write of the regions does not interrupt the acquisition.
  void set_regions(const std::vector<PhotRegion> &regions);
  std::vector<PhotRegion> get_regions();
  bool get_phot_subtract_background();
  void set_phot_subtract_background(bool on);

  // Ask for every photometry sample after last_frame_id.
  // The reply is [n_regions, n_records, records...]. One record is [frame_id, t_PC_ns, t_cam_ns, v_0 .. v_(n-1)].
  // t_PC_ns is the name of the server for a reading of the monotonic clock of the PC.
  PhotBatch get_phot_since(uint64_t last_frame_id);
};
