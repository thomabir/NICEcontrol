#pragma once

#include "client/nice_clock.h"
#include "core/Whiteboard.hpp"
#include "imgui.h"

// The distributed clock of the bus, and the estimate that gives its time for any time of the PC clock.
class ClockPanel {
 public:
  void Render(const Snapshot &snap) const {
    const ClockState &state = snap.clock;
    ImGui::Text("Health:   %s", state.dc_good ? "good, thus " NICE_CLOCK_PATH " gives t_DC to the other programs"
                                              : "no time of the bus, thus t_DC and the shared record are not good");
    if (!state.card_open) {
      ImGui::TextDisabled("The card is not open. The program tries once, at its start.");
      return;
    }
    ImGui::Text("Card:     AL state %d, %llu pairs, age %.1f ms, read span %.1f us", state.al_state,
                (unsigned long long)state.sample_count, state.age_ms, state.read_span_us);
    ImGui::Text("t_DC:     %lld ns", (long long)state.dc_ns);
    if (!state.locked) {
      ImGui::TextDisabled("Estimate: none. The maindevice does not distribute the clock.");
      return;
    }
    ImGui::Text("Offset:   t_DC - t_PC %+.6f s", 1e-9 * static_cast<double>(snap.time.t_DC - snap.time.t_PC));
    ImGui::Text("Estimate: %+.3f ppm +- %.0f ppb, uncertainty %.0f ns", state.rate_ppm, state.rate_sd_ppb,
                state.offset_sd_ns);
    ImGui::Text("          last error %+.0f ns, %llu pairs refused", state.error_ns,
                (unsigned long long)state.rejected_count);
  }
};
