#pragma once

#include "MetrologyReader.hpp"

struct Workers {
  MetrologyReader metrology_reader;

  Workers(SharedResources &resources) : metrology_reader(resources.metrology) {}
};
