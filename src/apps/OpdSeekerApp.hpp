#pragma once

#include "algorithms/ExtremumSeeker.hpp"
#include "core/Commands.hpp"
#include "core/Whiteboard.hpp"

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
  OpdSeekerApp(Whiteboard &whiteboard, CommandBox &command_box, double sample_time_s);

  void plan(Commands &command);

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
  double sample_timeout_s() const;

  OpdSeekerBlock blocked_by(const Commands &command, int region) const;
};
