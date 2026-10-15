#pragma once

#include <cstdint>

#include "core/Blackboard.hpp"
#include "core/Commands.hpp"
#include "core/Whiteboard.hpp"
#include "devices/TangoFlirCamInterface.hpp"

// The FLIR science camera.
// The photometry is the part that a control loop needs, so the core takes it in every cycle. The settings and the
// image are for the log and for the user interface, so the core takes them at a lower rate. This keeps the cycle
// short and keeps the photometry available even when no user interface runs.
class CameraApp {
 public:
  CameraApp(Whiteboard &whiteboard, Blackboard &blackboard, CommandBox &command_box)
      : wb(whiteboard), bb(blackboard), box(command_box) {}

  void sense() {
    if (!camera.is_connected()) {
      return;
    }

    take_photometry();
    take_settings();
    take_image();
  }

  void act(const CameraCommands &command) {
    if (command.connect != camera.is_connected()) {
      if (command.connect) {
        camera.connect();
        if (camera.is_connected()) {
          seed_from_camera();
        }
      } else {
        camera.disconnect();
      }
      wb.state.camera.connected = camera.is_connected();
      sent_valid = false;
      return;
    }
    if (!camera.is_connected()) {
      return;
    }

    if (sent_valid && command.device_command_count != sent.device_command_count) {
      camera.run_command(command.device_command);
    }
    if (sent_valid && command.record_count != sent.record_count) {
      camera.start_recording(command.record_frames);
    }
    if (sent_valid && command.background_count != sent.background_count) {
      camera.start_recording_background(command.background_frames);
    }

    // sent holds the settings of the camera from the connect on, thus a comparison against sent is a comparison
    // against the camera itself, and only a change of the commands goes out.
    if (command.settings_valid) {
      if (command.subtract_background != sent.subtract_background) {
        camera.set_phot_subtract_background(command.subtract_background);
      }
      if (!same_regions(command.regions, sent.regions)) {
        camera.set_regions(command.regions);
      }
      // Each of these restarts the stream, so they only go out on a change.
      if (command.framerate != sent.framerate) {
        camera.write_framerate(command.framerate);
      }
      if (command.integration_time_ms != sent.integration_time_ms) {
        camera.write_integration_time(command.integration_time_ms);
      }
      if (command.width != sent.width) {
        camera.write_width(command.width);
      }
      if (command.height != sent.height) {
        camera.write_height(command.height);
      }
      if (command.filename != sent.filename) {
        std::string filename = command.filename;
        camera.set_filename(filename);
      }
      if (command.sync_source != sent.sync_source) {
        camera.write_sync_source(command.sync_source);
      }
      if (command.sync_mode != sent.sync_mode) {
        camera.write_sync_mode(command.sync_mode);
      }
      if (command.sync_polarity != sent.sync_polarity) {
        camera.write_sync_polarity(command.sync_polarity);
      }
      if (command.sync_delay_us != sent.sync_delay_us) {
        camera.write_sync_delay_us(command.sync_delay_us);
      }
    }

    sent = command;
    sent_valid = true;
  }

 private:
  // The image comes in every cycle, which is above the screen refresh rate. Each setting takes one round trip per
  // stride, so the whole set is refreshed every kSettingsStride * kSettingsCount cycles.
  static constexpr uint64_t kSettingsStride = 10;
  static constexpr int kSettingsCount = 12;

  Whiteboard &wb;
  Blackboard &bb;
  CommandBox &box;
  TangoFlirCamInterface camera;

  uint64_t last_frame_id = 0;
  int settings_turn = 0;
  CameraCommands sent;
  bool sent_valid = false;

  void take_photometry() {
    const PhotBatch batch = camera.get_phot_since(last_frame_id);
    if (batch.samples.empty()) {
      return;
    }
    last_frame_id = batch.samples.back().frame_id;

    // The server stamps the frame when it arrives, thus the round trip of the Tango call stays out of the timestamp.
    for (const PhotSample &sample : batch.samples) {
      wb.phot.push({wb.clocks.stamp_from_monotonic(sample.t_mono), sample});
    }
    wb.state.camera.n_regions = batch.n_regions;
    wb.state.camera.values = batch.samples.back().values;
    wb.state.camera.frame_count += batch.samples.size();
  }

  // Read the whole set of settings, and start the commands and the record of what went out from it.
  //
  // This runs inside the connect, thus every cycle after it has settings_valid true and sent holds what the camera
  // holds. A command therefore never counts as sent before it goes out.
  void seed_from_camera() {
    CameraState &state = wb.state.camera;
    state.framerate = camera.read_framerate();
    state.integration_time_ms = camera.read_integration_time();
    state.width = camera.get_width();
    state.height = camera.get_height();
    state.filename = camera.get_filename();
    state.subtract_background = camera.get_phot_subtract_background();
    state.regions = camera.get_regions();
    state.sync_source = camera.get_sync_source();
    state.sync_mode = camera.get_sync_mode();
    state.sync_polarity = camera.get_sync_polarity();
    state.sync_delay_us = camera.get_sync_delay_us();
    state.device_commands = camera.get_commands();
    settings_turn = 0;

    // The camera reports the truth. The commands and the record of what went out both start from it, thus nothing
    // goes to the camera until someone asks for a change.
    sent.framerate = state.framerate;
    sent.integration_time_ms = state.integration_time_ms;
    sent.width = state.width;
    sent.height = state.height;
    sent.filename = state.filename;
    sent.subtract_background = state.subtract_background;
    sent.regions = state.regions;
    sent.sync_source = state.sync_source;
    sent.sync_mode = state.sync_mode;
    sent.sync_polarity = state.sync_polarity;
    sent.sync_delay_us = state.sync_delay_us;

    box.edit([&state](Commands &commands) {
      commands.camera.framerate = state.framerate;
      commands.camera.integration_time_ms = state.integration_time_ms;
      commands.camera.width = state.width;
      commands.camera.height = state.height;
      commands.camera.filename = state.filename;
      commands.camera.subtract_background = state.subtract_background;
      commands.camera.regions = state.regions;
      commands.camera.sync_source = state.sync_source;
      commands.camera.sync_mode = state.sync_mode;
      commands.camera.sync_polarity = state.sync_polarity;
      commands.camera.sync_delay_us = state.sync_delay_us;
      commands.camera.settings_valid = true;
    });
  }

  // The settings change rarely, so they take one round trip per stride and take turns. This keeps
  // the read off the critical path of the image, and no cycle carries the whole set. It refreshes what the panel
  // shows; the commands come from the seed and from whoever asks for a change.
  void take_settings() {
    if (wb.state.core.cycle % kSettingsStride != 0) {
      return;
    }

    CameraState &state = wb.state.camera;
    switch (settings_turn) {
      case 0:
        state.framerate = camera.read_framerate();
        break;
      case 1:
        state.integration_time_ms = camera.read_integration_time();
        break;
      case 2:
        state.width = camera.get_width();
        break;
      case 3:
        state.height = camera.get_height();
        break;
      case 4:
        state.filename = camera.get_filename();
        break;
      case 5:
        state.subtract_background = camera.get_phot_subtract_background();
        break;
      case 6:
        state.regions = camera.get_regions();
        break;
      case 7:
        state.sync_source = camera.get_sync_source();
        break;
      case 8:
        state.sync_mode = camera.get_sync_mode();
        break;
      case 9:
        state.sync_polarity = camera.get_sync_polarity();
        break;
      case 10:
        state.sync_delay_us = camera.get_sync_delay_us();
        break;
      default:
        state.device_commands = camera.get_commands();
        break;
    }

    settings_turn = (settings_turn + 1) % kSettingsCount;
  }

  void take_image() {
    // The time comes before the transfer, because the frame is older than the read.
    const Timestamp time = wb.clocks.stamp_now();
    if (sent.image_product == 1) {
      bb.camera_image.store({time, camera.get_image_bg_sub()});
    } else {
      bb.camera_image.store({time, camera.get_image()});
    }
  }

  static bool same_regions(const std::vector<PhotRegion> &a, const std::vector<PhotRegion> &b) {
    if (a.size() != b.size()) {
      return false;
    }
    for (size_t i = 0; i < a.size(); i++) {
      if (a[i].x0 != b[i].x0 || a[i].y0 != b[i].y0 || a[i].x1 != b[i].x1 || a[i].y1 != b[i].y1) {
        return false;
      }
    }
    return true;
  }
};
