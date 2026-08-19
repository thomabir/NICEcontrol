#pragma once

#include <cstdint>

#pragma pack(push, 1)
struct QpdData {
  float x1, x2;
  float y1, y2;
  float i1, i2;
};
#pragma pack(pop)
static_assert(sizeof(QpdData) == 24, "QpdData must be 24 bytes (packed)");
