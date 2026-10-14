#pragma once

#include <algorithm>
#include <array>

// The tip/tilt actuators of the lateral beam control, in the order of the arrays in the commands and in the state.
// Each actuator has its own local axes. The beam coordinates are common to all actuators. All angles are in mrad.
inline constexpr int kTipTiltActuators = 4;
inline constexpr std::array<const char *, kTipTiltActuators> kTipTiltActuatorNames = {"PI 1", "PI 2", "nF 1", "nF 2"};

namespace tiptilt {

using Vector2 = std::array<double, 2>;
using Matrix2 = std::array<Vector2, 2>;  // two rows

constexpr Vector2 multiply(const Matrix2 &m, const Vector2 &v) {
  return {m[0][0] * v[0] + m[0][1] * v[1], m[1][0] * v[0] + m[1][1] * v[1]};
}

constexpr double determinant(const Matrix2 &m) { return m[0][0] * m[1][1] - m[0][1] * m[1][0]; }

constexpr Matrix2 inverse(const Matrix2 &m) {
  const double d = determinant(m);
  return {{{m[1][1] / d, -m[0][1] / d}, {-m[1][0] / d, m[0][0] / d}}};
}

// The map from the local axes of each actuator to the beam coordinates:
//   (beam x, beam y) = kLocalToBeam[i] * (local axis 0, local axis 1)
// Row 0 gives beam x, and row 1 gives beam y. Column 0 is the effect of local axis 0, and column 1 is the effect of
// local axis 1. Any invertible matrix is valid, for example a rotation for a stage that moves the beam diagonally.
inline constexpr std::array<Matrix2, kTipTiltActuators> kLocalToBeam = {{
    // PI 1. The beam coordinates are at the entrance of the spatial filter collimator. The periscope inverts the
    // field, thus the axes are exchanged and both signs are inverted.
    {{{0.0, -1.0}, {-1.0, 0.0}}},
    // PI 2. The beam coordinates are at the entrance of the spatial filter collimator. The axes are exchanged.
    {{{0.0, 1.0}, {1.0, 0.0}}},
    // nF 1. The beam coordinates are the local axes.
    {{{1.0, 0.0}, {0.0, 1.0}}},
    // nF 2. The beam coordinates are the local axes.
    {{{1.0, 0.0}, {0.0, 1.0}}},
}};

static_assert(std::ranges::none_of(kLocalToBeam, [](const Matrix2 &m) { return determinant(m) == 0.0; }),
              "each map from the local axes to the beam coordinates must be invertible");

// The map from the beam coordinates to the local axes of each actuator: the inverse of kLocalToBeam.
inline constexpr std::array<Matrix2, kTipTiltActuators> kBeamToLocal = [] {
  std::array<Matrix2, kTipTiltActuators> maps{};
  for (int i = 0; i < kTipTiltActuators; i++) {
    maps[i] = inverse(kLocalToBeam[i]);
  }
  return maps;
}();

// The first setpoint of each actuator in the user interface, in local axes.
inline constexpr std::array<Vector2, kTipTiltActuators> kStartSetpoint = {{
    {1.0, 1.0},  // PI 1
    {1.0, 1.0},  // PI 2
    {4.0, 4.0},  // nF 1
    {4.0, 4.0},  // nF 2
}};

}  // namespace tiptilt

// The setpoint of one actuator, in local axes. The core sends nothing to an actuator until its setpoint is set, thus
// the actuators stay where they are when the program starts.
struct TipTiltSetpoint {
  bool set = false;
  tiptilt::Vector2 local_mrad{};

  bool operator==(const TipTiltSetpoint &) const = default;
};
