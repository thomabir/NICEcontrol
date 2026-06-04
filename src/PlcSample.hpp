#pragma once

#include <cstdint>

#include "QpdData.hpp"

#pragma pack(push, 1)
struct PlcSample {
  uint32_t sample_no;
  uint64_t timestamp_ns;  // time in ns since 2000-01-01
  float opd_um;
  float dl_pos_um;
  float dl_cmd_um;
  QpdData qpd1;
  QpdData qpd2;
  float ch1_v;
};
#pragma pack(pop)
static_assert(sizeof(PlcSample) == 76, "PlcSample must be 76 bytes (packed)");
