#pragma once

#include <algorithm>

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
// A seeker on another pair of a measurement and a plant input is another application of this shape.
class OpdSeekerApp {
 public:
  OpdSeekerApp(Whiteboard &whiteboard, CommandBox &command_box, double sample_time_s)
      : wb(whiteboard), box(command_box), phot_reader(whiteboard.phot.subscribe()), seeker(sample_time_s) {}

  void plan(Commands &command) {
    const OpdSeekerCommands &cfg = command.opd_seeker;
    const DitherSettings &dither_cfg = command.opd.dither;
    const int region = std::clamp(cfg.region, 0, kMaxPhotRegions - 1);

    if (cfg.run != seeker.running()) {
      if (cfg.run) {
        seeker.start(command.opd.setpoint_um);
      } else {
        seeker.stop();
      }
    }

    // The stream is drained in every cycle, whether or not the seeker runs, so that the user interface shows the
    // gradient while the demodulation phase is set.
    seeker.begin_step(cfg.seeker);
    Measurement<PhotSample> measurement;
    while (wb.phot.try_pop(phot_reader, measurement)) {
      seeker.add_sample(measurement.value.values[region], dither::phase_rad(measurement.time.t_DC, dither_cfg));
    }
    const ExtremumSeekerState &state = seeker.step(dither::amplitude(dither_cfg));

    if (state.running) {
      const float setpoint_um = static_cast<float>(state.output);
      command.opd.setpoint_um = setpoint_um;                                      // the act step of this cycle sends it
      box.edit([setpoint_um](Commands &c) { c.opd.setpoint_um = setpoint_um; });  // and it stays when the seeker stops
    }

    wb.state.opd_seeker.region = region;
    wb.state.opd_seeker.seeker = state;
    if (!state.running) {
      // The output it reports is the setpoint it would start from, and not the one of the run before.
      wb.state.opd_seeker.seeker.output = command.opd.setpoint_um;
    }
  }

 private:
  Whiteboard &wb;
  CommandBox &box;
  Consumer *phot_reader;
  ExtremumSeeker seeker;
};
