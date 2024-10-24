#pragma once

#include <array>

// N PI controllers, each with P and I gains, diagonal N x N system
template <int N>
class DiagPIController {
 public:
  DiagPIController() {
    Ps.fill(0.);
    Is.fill(0.);
    error_integrals.fill(0.);
  }

  void setPI(std::array<float, N> Ps, std::array<float, N> Is) {
    this->Ps = Ps;
    this->Is = Is;
  }

  void reset_state() { error_integrals.fill(0); }

  void reset_all() {
    error_integrals.fill(0);
    Ps.fill(0);
    Is.fill(0);
  }

  std::array<float, N> step(std::array<float, N> inputs) {
    std::array<float, N> outputs;
    for (int i = 0; i < N; i++) {
      error_integrals[i] += inputs[i];
      outputs[i] = Ps[i] * inputs[i] + Is[i] * error_integrals[i];
    }
    return outputs;
  }

 private:
  std::array<float, N> Ps;
  std::array<float, N> Is;
  std::array<float, N> error_integrals;
};