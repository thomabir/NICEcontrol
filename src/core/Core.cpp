#include "core/Core.hpp"

#include "apps/CameraApp.hpp"
#include "apps/ClockApp.hpp"
#include "apps/MetrologyApp.hpp"
#include "apps/OpdSeekerApp.hpp"
#include "apps/PlcApp.hpp"
#include "apps/TangoDeviceApp.hpp"
#include "apps/TipTiltApp.hpp"

struct Core::Apps {
  Apps(Whiteboard &wb, CommandBox &box)
      : clock(wb),
        metrology(wb),
        plc(wb),
        tiptilt(wb),
        camera(wb, box),
        opd_seeker(wb, box, 1e-3 * static_cast<double>(kCyclePeriod.count())),
        shutter("motor/shutter/2", wb.state.shutter),
        ndfilter("motor/ndfilter/1", wb.state.ndfilter),
        polarizer("motor/polarizer/1", wb.state.polarizer),
        chopper("motor/chopper/1", wb.state.chopper) {}

  ClockApp clock;
  MetrologyApp metrology;
  PlcApp plc;
  TipTiltApp tiptilt;
  CameraApp camera;
  OpdSeekerApp opd_seeker;
  TangoDeviceApp shutter;
  TangoDeviceApp ndfilter;
  TangoDeviceApp polarizer;  // the Elliptec rotation stage
  TangoDeviceApp chopper;    // the Thorlabs MC2000B optical chopper
};

Core::Core() : apps(std::make_unique<Apps>(wb, box)) {}

// The thread stops before the applications are destroyed, because it is declared after them.
Core::~Core() = default;

void Core::start() {
  if (thread.joinable()) {
    return;
  }
  thread = std::jthread([this](std::stop_token stop) { run(stop); });
}

void Core::request_stop() {
  if (thread.joinable()) {
    thread.request_stop();
    thread.join();
  }
}

void Core::run(std::stop_token stop) {
  apps->tiptilt.init();  // the stages take seconds to connect, so this happens before the first cycle

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

void Core::cycle() {
  Apps &a = *apps;
  Commands command = box.get();
  wb.state.time = wb.clocks.stamp_now();
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
  a.clock.sense();
  core.clock_ms = lap();
  a.metrology.sense();
  core.metrology_ms = lap();
  a.plc.sense();
  core.plc_ms = lap();
  a.tiptilt.sense();
  core.tiptilt_ms = lap();
  a.camera.sense();
  core.camera_ms = lap();
  a.shutter.sense();
  a.ndfilter.sense();
  a.polarizer.sense();
  a.chopper.sense();
  core.devices_ms = lap();

  // The seeker reads the photometry of this cycle and writes the OPD setpoint that the act step sends.
  a.opd_seeker.plan(command);
  core.opd_seeker_ms = lap();

  a.plc.act(command.opd);
  core.plc_ms += lap();
  a.tiptilt.act(command.tiptilt);
  core.tiptilt_ms += lap();
  a.camera.act(command.camera);
  core.camera_ms += lap();
  a.shutter.act(command.shutter);
  a.ndfilter.act(command.ndfilter);
  a.polarizer.act(command.polarizer);
  a.chopper.act(command.chopper);
  core.devices_ms += lap();

  core.cycle++;
  core.time_s = 1e-9 * static_cast<double>(wb.state.time.t_PC - t_PC_start);
  core.cycle_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  wb.publish();
}
