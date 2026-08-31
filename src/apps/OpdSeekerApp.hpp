#pragma once

#include <algorithm>
#include <cstddef>

#include "algorithms/Dither.hpp"
#include "algorithms/ExtremumSeeker.hpp"
#include "core/Commands.hpp"
#include "core/Whiteboard.hpp"
#include "data/PhotometryRegions.hpp"

// Extremum seeking on the optical path difference. It makes one photometry region as dark as it can.
//
// The PLC adds the dither to the plant input of the OPD loop, thus the delay line moves the OPD by a sine and the
// intensity of the region follows. ExtremumSeeker holds the algorithm and PlcApp holds the plant. This application
// only joins the two: it takes the photometry of the cycle, it gives each sample the phase that the dither had at
// the time of that sample, and it writes the output of the seeker into the OPD setpoint.
//
// It also holds the conditions of the run. The seeker moves the setpoint of a control loop, thus it does nothing but
// wind up to its limit when that loop is open, when the dither is off, when the clock that carries the phase is not
// good, or when the camera does not measure the region. The seeker only runs while all of them hold, and it stops
// and clears the command of the user when one of them goes.
//
// A seeker on another pair of a measurement and a plant input is another application of this shape.
class OpdSeekerApp {
 public:
  OpdSeekerApp(Whiteboard &whiteboard, CommandBox &command_box, double sample_time_s)
      : wb(whiteboard), box(command_box), phot_reader(whiteboard.phot.subscribe()), seeker(sample_time_s) {}

  void plan(Commands &command) {
    const OpdSeekerCommands &cfg = command.opd_seeker;
    const DitherSettings &dither_cfg = command.opd.dither;
    const int region = std::clamp(cfg.region, 0, kMaxPhotRegions - 1);
    const OpdSeekerBlock block = blocked_by(command, region);
    const bool run = cfg.run && block == kSeekerReady;

    if (cfg.run && !run) {
      box.edit([](Commands &c) { c.opd_seeker.run = false; });  // the button of the user interface follows this
    }
    if (run != seeker.running()) {
      if (run) {
        seeker.start(command.opd.setpoint_um);
      } else {
        seeker.stop();
      }
    }
    if (region != region_now) {
      region_now = region;
      seeker.reset_estimate();  // the measurement is another one, thus what the low pass holds is of no use
    }

    // The camera, and not the user, sets how long a gap in the photometry the seeker keeps its setpoint over.
    ExtremumSeekerConfig seeker_cfg = cfg.seeker;
    seeker_cfg.sample_timeout_s = static_cast<float>(sample_timeout_s());
    seeker.begin_step(seeker_cfg, dither::amplitude(dither_cfg));

    // The stream is drained in every cycle, whether or not the seeker runs, so that the user interface shows the
    // gradient while the demodulation phase is set.
    Measurement<PhotSample> measurement;
    while (wb.phot.try_pop(phot_reader, measurement)) {
      if (!measurement.time.dc_good) {
        continue;  // t_DC is a copy of t_PC there, and that phase is not the phase that the PLC had
      }
      seeker.add_sample(measurement.value.values[region], dither::phase_rad(measurement.time.t_DC, dither_cfg));
    }
    const ExtremumSeekerState &state = seeker.step();

    if (state.running) {
      const float setpoint_um = static_cast<float>(state.output);
      command.opd.setpoint_um = setpoint_um;                                      // the act step of this cycle sends it
      box.edit([setpoint_um](Commands &c) { c.opd.setpoint_um = setpoint_um; });  // and it stays when the seeker stops
    }

    wb.state.opd_seeker.region = region;
    wb.state.opd_seeker.run = run;
    wb.state.opd_seeker.block = block;
    wb.state.opd_seeker.seeker = state;
    if (!state.running) {
      // The output it reports is the setpoint it would start from, and not the one of the run before.
      wb.state.opd_seeker.seeker.output = command.opd.setpoint_um;
    }
  }

 private:
  static constexpr int kOpdClosedLoop = 3;
  static constexpr double kMinTimeout = 0.2;
  static constexpr double kTimeoutFrames = 5.0;

  Whiteboard &wb;
  CommandBox &box;
  Consumer *phot_reader;
  ExtremumSeeker seeker;
  int region_now = 0;

  // How long the seeker keeps its setpoint over a gap in the photometry. A few frames of the camera, and never
  // less than kMinTimeout, thus a camera of any frame rate has room for a frame that comes late.
  double sample_timeout_s() const {
    const double framerate = wb.state.camera.framerate;
    if (!(framerate > 0.0)) {
      return kMinTimeout;
    }
    return std::max(kMinTimeout, kTimeoutFrames / framerate);
  }

  OpdSeekerBlock blocked_by(const Commands &command, int region) const {
    if (!wb.state.opd.connected) {
      return kSeekerNoPlc;
    }
    if (command.opd.mode != kOpdClosedLoop) {
      return kSeekerOpenLoop;
    }
    if (!(dither::amplitude(command.opd.dither) > 0.0)) {
      return kSeekerNoDither;
    }
    if (!wb.state.clock.dc_good) {
      return kSeekerNoClock;
    }
    if (static_cast<size_t>(region) >= wb.state.camera.n_regions) {
      return kSeekerNoRegion;
    }
    return kSeekerReady;
  }
};
