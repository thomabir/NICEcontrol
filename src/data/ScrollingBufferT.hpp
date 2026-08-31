#pragma once

#include "data/PlotPoint.hpp"
#include "imgui.h"

// A scrolling buffer of plot points.

template <typename T, typename U>
struct ScrollingBufferT {
  int MaxSize;
  int Offset;
  ImVector<PlotPoint<T, U>> Data;
  ScrollingBufferT(int max_size = 70000) {
    MaxSize = max_size;
    Offset = 0;
    Data.reserve(MaxSize);
    // fill with zeros
    for (int i = 0; i < MaxSize; i++) {
      Data.push_back(PlotPoint<T, U>(0, 0));
    }
  }
  void AddPoint(T x, U y) {
    if (Data.size() < MaxSize)
      Data.push_back(PlotPoint<T, U>(x, y));
    else {
      Data[Offset] = PlotPoint<T, U>(x, y);
      Offset = (Offset + 1) % MaxSize;
    }
  }
  void Erase() {
    if (Data.size() > 0) {
      Data.shrink(0);
      Offset = 0;
    }
  }

  // get last n points
  ImVector<PlotPoint<T, U>> GetLastN(int n) {
    ImVector<PlotPoint<T, U>> last_n;
    last_n.reserve(n);
    for (int i = 0; i < n; i++) {
      last_n.push_back(Data[(Offset - n + i + MaxSize) % MaxSize]);
    }
    return last_n;
  }
};
