#pragma once
#include <vector>

template <typename T>
struct Image {
  std::vector<T> data;
  unsigned int width;
  unsigned int height;
};
