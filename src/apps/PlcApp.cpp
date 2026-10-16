#include "apps/PlcApp.hpp"

#include <cstddef>
#include <exception>
#include <iostream>

#include "devices/PlcConnection.hpp"

namespace {

const PlcConnection::Config kPlcConfig{"192.168.88.21", {5, 168, 39, 125, 1, 1}, {1, 2, 3, 4, 5, 6}};

// The PLC holds a ring of the most recent samples. STREAM_BUF_SIZE in the PLC program sets its length.
constexpr size_t kPlcBufferSize = 512;

#pragma pack(push, 1)
struct PlcBuffer {
  uint32_t buffer_no;  // index of the most recent sample
  PlcSample samples[kPlcBufferSize];
};
#pragma pack(pop)
static_assert(sizeof(PlcBuffer) == 4 + sizeof(PlcSample) * kPlcBufferSize, "PlcBuffer size mismatch");

}  // namespace

PlcApp::PlcApp(Whiteboard &whiteboard) : wb(whiteboard) {}

PlcApp::~PlcApp() = default;

void PlcApp::sense() {
  if (!connect()) {
    return;
  }

  PlcBuffer buffer;
  try {
    buffer = plc->read<PlcBuffer>("MAIN.send_buffer");
  } catch (const std::exception &error) {
    std::cerr << "PlcApp: read failed: " << error.what() << std::endl;
    drop_connection();
    return;
  }

  const uint32_t oldest = (buffer.buffer_no + 1) % kPlcBufferSize;
  size_t first = 0;
  if (!have_last_sample) {
    take(buffer.samples[oldest]);
    have_last_sample = true;
    first = 1;
  }

  for (size_t j = first; j < kPlcBufferSize; j++) {
    const PlcSample &sample = buffer.samples[(oldest + j) % kPlcBufferSize];
    if (sample.sample_no <= last_sample_no) {
      continue;
    }
    if (sample.sample_no != last_sample_no + 1) {
      wb.state.opd.gaps++;
    }
    take(sample);
  }
}

void PlcApp::act(const OpdCommands &command) {
  // The user interface reads the setpoint back from here, thus it is written whether or not the PLC answers.
  wb.state.opd.setpoint_um = command.setpoint_um;

  if (!plc) {
    return;
  }

  try {
    if (!sent_valid || command.mode != sent.mode) {
      plc->write<int16_t>("MAIN.opd_mode", static_cast<int16_t>(command.mode));
    }
    if (!sent_valid || command.setpoint_um != sent.setpoint_um) {
      plc->write<float>("MAIN.opd_setpoint_um", command.setpoint_um);
    }
    if (!sent_valid || command.open_loop_cmd_um != sent.open_loop_cmd_um) {
      plc->write<float>("MAIN.opd_open_loop_cmd_um", command.open_loop_cmd_um);
    }
    if (!sent_valid || command.kp != sent.kp) {
      plc->write<float>("MAIN.opd_kp", command.kp);
    }
    if (!sent_valid || command.ki != sent.ki) {
      plc->write<float>("MAIN.opd_ki", command.ki);
    }
    if (!sent_valid || command.dither.mode != sent.dither.mode) {
      plc->write<int16_t>("MAIN.opd_dither_pl_mode", static_cast<int16_t>(command.dither.mode));
    }
    if (!sent_valid || command.dither.period_ns != sent.dither.period_ns) {
      plc->write<uint64_t>("MAIN.opd_dither_pl_period_ns", static_cast<uint64_t>(command.dither.period_ns));
    }
    if (!sent_valid || command.dither.phase_rad != sent.dither.phase_rad) {
      plc->write<float>("MAIN.opd_dither_pl_phase_at_tzero_rad", command.dither.phase_rad);
    }
    if (!sent_valid || command.dither.amplitude != sent.dither.amplitude) {
      plc->write<float>("MAIN.opd_dither_pl_amplitude_um", command.dither.amplitude);
    }
    if (sent_valid && command.reset_unwrap_count != sent.reset_unwrap_count) {
      plc->write<bool>("MAIN.reset_unwrap", true);
      plc->write<bool>("MAIN.reset_unwrap", false);
    }
  } catch (const std::exception &error) {
    std::cerr << "PlcApp: write failed: " << error.what() << std::endl;
    drop_connection();
    return;
  }

  sent = command;
  sent_valid = true;
}

bool PlcApp::connect() {
  if (plc) {
    return true;
  }
  if (--retry_countdown > 0) {
    return false;
  }
  retry_countdown = kRetryCycles;

  try {
    plc = std::make_unique<PlcConnection>(kPlcConfig);
  } catch (const std::exception &error) {
    std::cerr << "PlcApp: cannot connect: " << error.what() << std::endl;
    return false;
  }
  wb.state.opd.connected = true;
  return true;
}

void PlcApp::drop_connection() {
  plc.reset();
  wb.state.opd.connected = false;
  have_last_sample = false;
  sent_valid = false;
  retry_countdown = kRetryCycles;
}

void PlcApp::take(const PlcSample &sample) {
  const Timestamp time = wb.clocks.stamp_from_t_DC(sample.timestamp_ns);
  wb.plc.push({time, sample});
  last_sample_no = sample.sample_no;

  OpdState &state = wb.state.opd;
  state.sample_no = sample.sample_no;
  state.time = time;
  state.opd_um = sample.opd_um;
  state.dl_pos_um = sample.dl_pos_um;
  state.dl_cmd_um = sample.dl_cmd_um;
  state.sample_count++;
}
