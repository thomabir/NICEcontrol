#include "apps/OpdSeekerApp.hpp"

#include <algorithm>
#include <cstddef>

#include "algorithms/Dither.hpp"
#include "data/PhotometryRegions.hpp"

OpdSeekerApp::OpdSeekerApp(Whiteboard &whiteboard, CommandBox &command_box, double sample_time_s)
    : wb(whiteboard), box(command_box), phot_reader(whiteboard.phot.subscribe()), seeker(sample_time_s) {}

void OpdSeekerApp::plan(Commands &command) {
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

double OpdSeekerApp::sample_timeout_s() const {
  const double framerate = wb.state.camera.framerate;
  if (!(framerate > 0.0)) {
    return kMinTimeout;
  }
  return std::max(kMinTimeout, kTimeoutFrames / framerate);
}

OpdSeekerBlock OpdSeekerApp::blocked_by(const Commands &command, int region) const {
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
