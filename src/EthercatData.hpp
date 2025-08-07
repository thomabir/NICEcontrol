#pragma once

struct EthercatData {
 public:
  // meta
  uint buffer_index;     // index of the buffer, used to identify the data
  int64_t timestamp_us;  // timestamp in microseconds

  // delay line
  float dl_position_cmd;
  float dl_position_meas;

  // metrology
  float metr_opd_nm_unwrapped;
  std::array<float, 12> metr_qpd;  // QPD measurements for all 12 channels

  // TODO: Gateway to monitor control loop flags and configurations:
  // Control flag
  // Phase unwrap flag
  // Init EtherCAT flag
  // Save data flag
  // Start SysID flag
  // SysID running flag
  // Reset IIR memory flag
  // setpoint
  // kp gain
  // ki gain
};
