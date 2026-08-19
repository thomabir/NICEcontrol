#pragma once

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>

#include "core/Commands.hpp"
#include "core/Whiteboard.hpp"
#include "data/PlcSample.hpp"
#include "devices/PlcConnection.hpp"

// The PLC holds a ring of the most recent samples. STREAM_BUF_SIZE in the PLC program sets its length.
static constexpr size_t kPlcBufferSize = 512;

#pragma pack(push, 1)
struct PlcBuffer {
  uint32_t buffer_no;  // index of the most recent sample
  PlcSample samples[kPlcBufferSize];
};
#pragma pack(pop)
static_assert(sizeof(PlcBuffer) == 4 + sizeof(PlcSample) * kPlcBufferSize, "PlcBuffer size mismatch");

// Reads the metrology and delay line samples from the PLC, and sends the optical path difference settings to it.
// The control loop itself runs on the PLC. This application only relays the settings.
class PlcApp {
 public:
  explicit PlcApp(Whiteboard &whiteboard) : wb(whiteboard) {}

  void sense() {
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

  void act(const OpdCommands &command) {
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

 private:
  Whiteboard &wb;
  PlcConnection::Config config{"192.168.88.21", {5, 168, 39, 125, 1, 1}, {1, 2, 3, 4, 5, 6}};
  std::unique_ptr<PlcConnection> plc;

  int retry_countdown = 1;
  uint32_t last_sample_no = 0;
  bool have_last_sample = false;

  OpdCommands sent;
  bool sent_valid = false;

  // A connection attempt is slow when the PLC is absent, so it happens once per retry interval and not once per cycle.
  static constexpr int kRetryCycles = 100;

  bool connect() {
    if (plc) {
      return true;
    }
    if (--retry_countdown > 0) {
      return false;
    }
    retry_countdown = kRetryCycles;

    try {
      plc = std::make_unique<PlcConnection>(config);
    } catch (const std::exception &error) {
      std::cerr << "PlcApp: cannot connect: " << error.what() << std::endl;
      return false;
    }
    wb.state.opd.connected = true;
    return true;
  }

  void drop_connection() {
    plc.reset();
    wb.state.opd.connected = false;
    have_last_sample = false;
    sent_valid = false;
    retry_countdown = kRetryCycles;
  }

  // The PLC is a device of the bus, thus its own timestamp is the distributed clock and the PC time follows from it.
  void take(const PlcSample &sample) {
    const Timestamp time = wb.time.stamp_from_t_DC(static_cast<int64_t>(sample.timestamp_ns));
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
};
