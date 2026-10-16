#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "core/Commands.hpp"
#include "core/Whiteboard.hpp"

class TangoFlirCamInterface;

// The FLIR science camera.
// The photometry is the part that a control loop needs, so the core takes it in every cycle. The settings and the
// image are for the log and for the user interface, so the core takes them at a lower rate. This keeps the cycle
// short and keeps the photometry available even when no user interface runs.
class CameraApp {
 public:
  CameraApp(Whiteboard &whiteboard, CommandBox &command_box);
  ~CameraApp();

  void sense();
  void act(const CameraCommands &command);

 private:
  // The image comes in every cycle, which is above the screen refresh rate. Each setting takes one round trip per
  // stride, so the whole set is refreshed every kSettingsStride * kSettingsCount cycles.
  static constexpr uint64_t kSettingsStride = 10;
  static constexpr int kSettingsCount = 12;

  Whiteboard &wb;
  CommandBox &box;
  std::unique_ptr<TangoFlirCamInterface> camera;

  uint64_t last_frame_id = 0;
  int settings_turn = 0;
  CameraCommands sent;
  bool sent_valid = false;

  void take_photometry();

  // Read the whole set of settings, and start the commands and the record of what went out from it.
  //
  // This runs inside the connect, thus every cycle after it has settings_valid true and sent holds what the camera
  // holds. A command therefore never counts as sent before it goes out.
  void seed_from_camera();

  // The settings change rarely, so they take one round trip per stride and take turns. This keeps
  // the read off the critical path of the image, and no cycle carries the whole set. It refreshes what the panel
  // shows; the commands come from the seed and from whoever asks for a change.
  void take_settings();

  void take_image();

  static bool same_regions(const std::vector<PhotRegion> &a, const std::vector<PhotRegion> &b);
};
