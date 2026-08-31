#pragma once

// One point of a plot in the user interface.
template <typename T, typename U>
struct PlotPoint {
  T time;
  U value;
};
