#pragma once

#include <cstdint>

#pragma pack(push, 1)
struct PlcSample {
  uint32_t sample_no;
  uint64_t timestamp_ns;  // time in ns since 2000-01-01
  float opd_um;
  float dl_pos_um;
  float dl_cmd_um;
  float ch1_v;
};
#pragma pack(pop)
static_assert(sizeof(PlcSample) == 28, "PlcSample must be 28 bytes (packed)");
