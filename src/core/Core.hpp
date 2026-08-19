#pragma once

#include <chrono>
#include <thread>

#include "apps/CameraApp.hpp"
#include "apps/ClockApp.hpp"
#include "apps/MetrologyApp.hpp"
#include "apps/PlcApp.hpp"
#include "apps/TangoDeviceApp.hpp"
#include "apps/TipTiltApp.hpp"
#include "core/Blackboard.hpp"
#include "core/Commands.hpp"
#include "core/Whiteboard.hpp"
#include "utils.hpp"

// The control core. It runs on one thread at a fixed cycle period and it is independent of the user interface.
//
// Each cycle runs three steps in order:
//   sense  every application reads its hardware and writes what it found on the whiteboard
//   plan   every application decides what it wants, and writes that on the whiteboard
//   act    every application sends its commands to its own hardware
// Only the act step reaches an actuator, so each actuator takes at most one command per cycle.
//
// The core steers the modes and leaves the details to the applications.
class Core {
 public:
  static constexpr std::chrono::milliseconds kCyclePeriod{10};

  Core()
      : clock(wb),
        metrology(wb),
        plc(wb),
        tiptilt(wb),
        camera(wb, bb, box),
        shutter("motor/shutter/2", wb.state.shutter),
        ndfilter("motor/ndfilter/1", wb.state.ndfilter) {}

  void start() {
    if (thread.joinable()) {
      return;
    }
    thread = std::jthread([this](std::stop_token stop) { run(stop); });
  }

  void request_stop() {
    if (thread.joinable()) {
      thread.request_stop();
      thread.join();
    }
  }

  Whiteboard &whiteboard() { return wb; }
  Blackboard &blackboard() { return bb; }
  CommandBox &commands() { return box; }

 private:
  Whiteboard wb;
  Blackboard bb;
  CommandBox box;

  ClockApp clock;
  MetrologyApp metrology;
  PlcApp plc;
  TipTiltApp tiptilt;
  CameraApp camera;
  TangoDeviceApp shutter;
  TangoDeviceApp ndfilter;

  std::jthread thread;

  void run(std::stop_token stop) {
    tiptilt.init();  // the stages take seconds to connect, so this happens before the first cycle

    auto next = std::chrono::steady_clock::now();
    while (!stop.stop_requested()) {
      next += kCyclePeriod;
      cycle();
      const auto now = std::chrono::steady_clock::now();
      if (now > next) {
        wb.state.core.overruns++;
        next = now;
      } else {
        std::this_thread::sleep_until(next);
      }
    }
  }

  void cycle() {
    const Commands command = box.get();
    wb.state.time = wb.time.stamp_now();
    const auto start = std::chrono::steady_clock::now();
    auto mark = start;
    auto lap = [&mark]() {
      const auto now = std::chrono::steady_clock::now();
      const double ms = std::chrono::duration<double, std::milli>(now - mark).count();
      mark = now;
      return ms;
    };

    CoreState &core = wb.state.core;

    // The clock comes first, so that the timestamps of this cycle use the newest pair of the two clocks.
    clock.sense();
    core.clock_ms = lap();
    metrology.sense();
    core.metrology_ms = lap();
    plc.sense();
    core.plc_ms = lap();
    tiptilt.sense();
    core.tiptilt_ms = lap();
    camera.sense();
    core.camera_ms = lap();
    shutter.sense();
    ndfilter.sense();
    core.devices_ms = lap();

    // No application needs a plan step yet. A control loop that needs one puts it here.

    plc.act(command.opd);
    core.plc_ms += lap();
    tiptilt.act(command.tiptilt);
    core.tiptilt_ms += lap();
    camera.act(command.camera);
    core.camera_ms += lap();
    shutter.act(command.shutter);
    ndfilter.act(command.ndfilter);
    core.devices_ms += lap();

    core.cycle++;
    core.time_s = utils::getTime();
    core.cycle_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    wb.publish();
  }
};
